#include <WiFi.h>
#include <PubSubClient.h>

// ===================== Wi-Fi/MQTT =====================
#define WIFI_SSID      "xxx"
#define WIFI_PASSWORD  "xxx"

#define MQTT_BROKER    "homeassistant.local"  // ou IP do broker, ex "192.168.15.35"
#define MQTT_PORT      1883
#define MQTT_USER      "xxx"
#define MQTT_PASS      "xxx"

// ===================== Dispositivo =====================
const char* DEVICE_NAME     = "esp32c3-bomba-hidroponia";
const char* DEVICE_FRIENDLY = "Bomba Hidroponia";

String base       = "homeassistant";
String availTopic = String("home/") + DEVICE_NAME + "/status";
String stateTopic = String("home/") + DEVICE_NAME + "/state";
String cmdTopic   = String("home/") + DEVICE_NAME + "/set";

WiFiClient espClient;
PubSubClient mqtt(espClient);

const int RELAY_PIN = 4;   // ajuste para o pino que você ligou o relé
bool relayOn = false;

char payload[256];

// ===================== Wi-Fi =====================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("WiFi connect ");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    Serial.print(".");
    delay(250);
  }

  Serial.println(WiFi.status() == WL_CONNECTED ? " OK" : " TIMEOUT");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI ");
    Serial.println(WiFi.RSSI());
  }
}

// ===================== MQTT Discovery =====================
void publishState() {
  const char* st = relayOn ? "ON" : "OFF";
  mqtt.publish(stateTopic.c_str(), st, true);
}

void publishDiscovery() {
  String devObj = String("\"dev\":{") +
                  "\"ids\":[\"" + String(DEVICE_NAME) + "\"]," +
                  "\"name\":\"" + String(DEVICE_FRIENDLY) + "\"," +
                  "\"mdl\":\"ESP32+RELE\",\"mf\":\"Andre\"}";

  String switchCfgTopic = base + "/switch/" + DEVICE_NAME + "/config";

  String cfg = String("{") +
               "\"name\":\""      + String(DEVICE_FRIENDLY) + "\"," +
               "\"uniq_id\":\""   + String(DEVICE_NAME) + "_switch\"," +
               "\"cmd_t\":\""     + cmdTopic   + "\"," +
               "\"stat_t\":\""    + stateTopic + "\"," +
               "\"avty_t\":\""    + availTopic + "\"," +
               "\"pl_on\":\"ON\"," +
               "\"pl_off\":\"OFF\"," +
               devObj +
               "}";

  mqtt.publish(switchCfgTopic.c_str(), cfg.c_str(), true);
}

// ===================== MQTT =====================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.toUpperCase();

  Serial.print("CMD ");
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(msg);

  if (String(topic) == cmdTopic) {
    if (msg == "ON") {
      relayOn = true;
      // relé ativo em nível baixo
      digitalWrite(RELAY_PIN, LOW);   // ON físico
      publishState();
    } else if (msg == "OFF") {
      relayOn = false;
      digitalWrite(RELAY_PIN, HIGH);  // OFF físico
      publishState();
    }
  }
}

bool connectMQTT(uint16_t keepAlive = 60) {
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setKeepAlive(keepAlive);
  mqtt.setBufferSize(512);
  mqtt.setCallback(mqttCallback);

  String clientId = String(DEVICE_NAME) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  Serial.print("MQTT connect ");
  bool ok = mqtt.connect(
    clientId.c_str(),
    MQTT_USER, MQTT_PASS,
    availTopic.c_str(), 0, true, "offline", true
  );

  if (ok) {
    Serial.println(" OK");
    mqtt.publish(availTopic.c_str(), "online", true);
    publishDiscovery();
    mqtt.subscribe(cmdTopic.c_str());
    publishState();
  } else {
    Serial.print(" fail rc=");
    Serial.println(mqtt.state());
  }
  return ok;
}

// backoff progressivo para reconexão
uint32_t nextRetryAt = 0;
uint16_t backoffSec  = 2;

void ensureConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!mqtt.connected()) {
    if (millis() >= nextRetryAt) {
      bool ok = connectMQTT();
      if (!ok) {
        backoffSec = min<uint16_t>(backoffSec * 2, 60);
        nextRetryAt = millis() + backoffSec * 1000UL;
        Serial.print("Retry in ");
        Serial.print(backoffSec);
        Serial.println("s");
      } else {
        backoffSec = 2;
        nextRetryAt = 0;
      }
    }
  }
}

// ===================== setup/loop =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(RELAY_PIN, OUTPUT);
  // estado inicial: relé desligado
  relayOn = false;
  digitalWrite(RELAY_PIN, HIGH);  // OFF físico para relé ativo em LOW

  connectWiFi();
  connectMQTT();
}

void loop() {
  ensureConnections();
  mqtt.loop();

  delay(50);
}
