#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#ifdef ESP32
  #include <esp_task_wdt.h>
  #include <ESPmDNS.h>
#else
  #include <ESP8266mDNS.h>
  #include <ESP8266TrueRandom.h>
#endif

#define MAX_SRV_CLIENTS 4
#define RXBUFFERSIZE 1024
#define TXBUFFERSIZE 512
#define STACK_PROTECTOR  512 // bytes
#define HOSTNAME "esp-eBus"
#define RESET_PIN 3  // RX pin
#define RESET_TIMEOUT 1000

#ifndef TX_DISABLE_PIN
#define TX_DISABLE_PIN 5
#endif

WiFiServer wifiServer(3333);
WiFiServer wifiServerRO(3334);
WiFiServer statusServer(5555);
WiFiClient serverClients[MAX_SRV_CLIENTS];
WiFiClient serverClientsRO[MAX_SRV_CLIENTS];

unsigned long last_comms;

unsigned char rxBuf[RXBUFFERSIZE];
unsigned char txBuf[TXBUFFERSIZE];

int random_ch() {
#ifdef ESP32
  return 6;
#elif defined(ESP8266)
  return ESP8266TrueRandom.random(1, 13);
#endif
}

void wdt_start() {
#ifdef ESP32
  esp_task_wdt_init(6, true);
#elif defined(ESP8266)
  ESP.wdtDisable();
#endif
}

void wdt_feed() {
#ifdef ESP32
  esp_task_wdt_reset();
#elif defined(ESP8266)
  ESP.wdtFeed();
#else
#error UNKNOWN PLATFORM
#endif
}

inline void disableTX() {
  digitalWrite(TX_DISABLE_PIN, HIGH);
}

inline void enableTX() {
  digitalWrite(TX_DISABLE_PIN, LOW);
}

void reset() {
  disableTX();
  pinMode(TX_DISABLE_PIN, INPUT_PULLUP);
  ESP.restart();
}


/**
 * @brief Reset the Wi-Fi settings, then restart.
 */
void reset_config() {
  WiFiManager wifiManager(Serial1);  // Send debug on Serial1
  wifiManager.resetSettings();
  reset();
}

void setup() {
#ifdef ESP32
  Serial1.begin(115200, SERIAL_8N1, 8, 10);
#elif defined(ESP8266)
  Serial1.begin(115200);
#endif
  Serial1.setDebugOutput(true);

  // Disable as soon as possible unavoidable early data emission by the ESP
  // on the serial interface
  disableTX();
  pinMode(TX_DISABLE_PIN, OUTPUT);

  // check if RX being hold low and reset
  pinMode(RESET_PIN, INPUT_PULLUP);
  unsigned long resetStart = millis();
  while(digitalRead(RESET_PIN) == LOW) {
    if (millis() > resetStart + RESET_TIMEOUT) {
      reset_config();
    }
  }

  Serial.setRxBufferSize(RXBUFFERSIZE);
  Serial.begin(2400);

  WiFi.enableAP(false);
  WiFiManager wifiManager(Serial1); // Send debug on Serial1
  wifiManager.setHostname(HOSTNAME);
  wifiManager.setConfigPortalTimeout(120);
  wifiManager.setWiFiAPChannel(random_ch());
  wifiManager.autoConnect(HOSTNAME);

  wifiServer.begin();
  wifiServerRO.begin();
  statusServer.begin();

  ArduinoOTA.begin();

  MDNS.end();
  MDNS.begin(HOSTNAME);

  wdt_start();

  last_comms = millis();
}


/**
 * @brief Handle new TCP client on the status server; then send it the uptime in ms.
 */
bool handleStatusServerRequests() {
  if (!statusServer.hasClient())
    return false;

  WiFiClient client = statusServer.available();

  String inputString;
  inputString.reserve(20);
  while (client_data_len) {
    char inChar = client.read();
    if (inChar != '\n') {
      // end of command not reached
      inputString += inChar;
      client_data_len = client.available();
      continue;
    }

    if (strncmp(inputString.c_str(), "reset", 5) == 0) {
        reset_config();
    }
    // No other command expected
    break;
  }

  if (client.availableForWrite() >= 1) {
    // Send the uptime
    client.println(millis());
    client.flush();
    client.stop();
  }
  return true;
}


/**
 * @brief Handle new TCP client for the given server
 *
 * @note If the server has no free slot, client is rejected &
 *      warned with a 'busy' message.
 */
bool handleNewClient(WiFiServer &server, WiFiClient clients[]) {
  if (!server.hasClient())
    return false;

  // Find free/disconnected slot
  int i;
  for (i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (!clients[i]) { // equivalent to !serverClients[i].connected()
      clients[i] = server.available();
      clients[i].setNoDelay(true);
      break;
    }
  }

  // No free/disconnected slot so reject
  if (i == MAX_SRV_CLIENTS) {
    server.available().println("busy");
    // hints: server.available() is a WiFiClient with short-term scope
    // when out of scope, a WiFiClient will
    // - flush() - all data will be sent
    // - stop() - automatically too
  }

  return true;
}


/**
 * @brief Push RX buffer available data to all the given TCP clients
 */
void pushDataToClients(WiFiClient clients[], size_t &data_length) {
  for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
    // if client.availableForWrite() was 0 (congested)
    // and increased since then,
    // ensure write space is sufficient:
    if (clients[i].availableForWrite() >= 1) {
      clients[i].write(rxBuf, data_length);
      last_comms = millis();
    }
  }
}


void loop() {
  ArduinoOTA.handle();

#ifdef ESP8266
  MDNS.update();
#endif

  wdt_feed();

  // Restart conditions
  if (WiFi.status() != WL_CONNECTED) {
    reset();
  }
  if (millis() > last_comms + 200*1000) {
    reset();
  }

  // Check & handle new client on the status server
  handleStatusServerRequests();

  // Check if there are any new clients on the servers
  if (handleNewClient(wifiServer, serverClients))
      enableTX();
  handleNewClient(wifiServerRO, serverClientsRO);

  // Check TCP clients for available data and send it to the eBUS
  for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
    size_t client_data_len = serverClients[i].available();
    size_t authorized_len = (client_data_len > TXBUFFERSIZE) ? TXBUFFERSIZE : client_data_len;

    while (authorized_len && Serial.availableForWrite() > 0) {
      size_t ret = serverClients[i].readBytes(txBuf, authorized_len);

      if (ret < authorized_len)
        // Throw out the received data
        continue;
      Serial.write(txBuf, authorized_len);

      client_data_len = serverClients[i].available();
      authorized_len = (client_data_len > TXBUFFERSIZE) ? TXBUFFERSIZE : client_data_len;
    }
  }

  // Check the UART bus for available data and send it to the clients
  size_t ebus_data_len = Serial.available();
  if (ebus_data_len) {
    size_t authorized_len = (ebus_data_len > RXBUFFERSIZE) ? RXBUFFERSIZE : ebus_data_len;
    size_t ret = Serial.readBytes(rxBuf, authorized_len);

    if (ret < authorized_len)
      // Throw out the received data
      return;

    pushDataToClients(serverClients, authorized_len);
    pushDataToClients(serverClientsRO, authorized_len);
  }
}
