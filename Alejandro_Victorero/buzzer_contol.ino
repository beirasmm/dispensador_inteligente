// Envio de por MQTT los datos del sensor de proximidad ALS Proximity Sensor
// Control de buzzer y LED integrado medinte MQTT

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <APDS9930.h>

const char* ssid = "ssid";
const char* password = "pass";

const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;

const char* topic_cmd  = "devices/NAPIoT-P2-USC2/cmd";
const char* topic_data = "devices/NAPIoT-P2-USC2/data";


#define LED_PIN        LED_BUILTIN
#define BUZZER_PIN     9
#define APDS9930_INT   2   

#define PROX_INT_HIGH 600
#define PROX_INT_LOW  0

APDS9930 apds;
volatile bool isr_flag = false;

uint16_t proximity_data = 0;
float ambient_light = 0;
uint16_t ch0 = 0;
uint16_t ch1 = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectada");
  Serial.println(WiFi.localIP());
}

void blinkAndBeep(int times) {
  // control de buzzer y led
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 1000);
    delay(200);

    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
    delay(200);
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (uint8_t i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("MQTT recibido: ");
  Serial.println(msg);

  if (String(topic) == topic_cmd) {
    if (msg == "1") {
      blinkAndBeep(3);
    } 
    else if (msg == "0") {
      digitalWrite(LED_PIN, LOW);
      noTone(BUZZER_PIN);
    }
  }
}


void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando MQTT...");
    if (client.connect("NAPIoT-ESP32")) {
      Serial.println("OK");
      client.subscribe(topic_cmd);
    } else {
      Serial.print("Error: ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void IRAM_ATTR interruptRoutine() {
  isr_flag = true;
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(APDS9930_INT, INPUT);

  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // ===== SENSOR APDS9930 =====
  Wire.begin();
  attachInterrupt(digitalPinToInterrupt(APDS9930_INT), interruptRoutine, FALLING);

  if (!apds.init()) {
    Serial.println("Error APDS9930");
    while (1);
  }

  apds.setProximityGain(PGAIN_2X);
  apds.setProximityIntLowThreshold(PROX_INT_LOW);
  apds.setProximityIntHighThreshold(PROX_INT_HIGH);
  apds.enableProximitySensor(true);
  apds.enableLightSensor(false);

  Serial.println("Sistema listo");
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (isr_flag) {
    isr_flag = false;

    apds.readProximity(proximity_data);
    apds.readAmbientLightLux(ambient_light);
    apds.readCh0Light(ch0);
    apds.readCh1Light(ch1);

    char payload[128];
    sprintf(payload,
            "{\"prox\":%u,\"lux\":%.2f,\"ch0\":%u,\"ch1\":%u}",
            proximity_data, ambient_light, ch0, ch1);

    client.publish(topic_data, payload);
    Serial.println(payload);

    apds.clearProximityInt();
  }
}
