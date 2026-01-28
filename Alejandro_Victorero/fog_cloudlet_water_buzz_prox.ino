#define DUMP_REGS

#include <Wire.h>
#include <APDS9930.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "ssid";
const char* password = "password";

const char* mqtt_fog    = "192.168.1.15";
const char* mqtt_cloud = "192.168.1.55";
const int   mqtt_port  = 1883;

bool usingFog = true;
unsigned long lastFogRetry = 0;
const unsigned long FOG_RETRY_INTERVAL = 10000; // intentar volver al Fog cada 10s

const char* topic_sub_cmd   = "buzzer";
const char* topic_pub_cmd   = "sensor/water";
const char* topic_pub_prox = "sensor/proximity";


// ===================== HARDWARE =====================
#define BUZZER_PIN 12 // GPIO 12
#define WATERSENSOR_PIN 11 // GPIO 11
#define APDS9930_INT  4

WiFiClient espClient;
PubSubClient client(espClient);

// ===================== INITIAL CONFIG =====================
bool buzzerState = false; // estado del buzzer
int buzzerFreqLow = 400;
int buzzerFreqHigh = 800;

unsigned long lastSend = 0; // timestamp ultimo envio sensor de agua
const unsigned long interval = 2000; // enviar metricas cada 2 seg del sensor de agua

// APDS constants
#define PROX_INT_HIGH 200 // umbral proximidad minimo
#define PROX_INT_LOW  0

APDS9930 apds = APDS9930();

volatile bool isr_flag = false;
uint16_t proximity_data = 0;
bool proxState = false; // estado para saber si hay un objeto proximo
#define PROX_ON_THRESHOLD   420
#define PROX_OFF_THRESHOLD  380
const unsigned long PROX_STABLE_TIME = 1000; // tiempo minimo para aceptar un cambio de proximidad
unsigned long proxCandidateSince = 0; // timestamp ultimo envio proximidad
bool proxCandidateState = false;
unsigned long lastProxEvent = 0;
const unsigned long PROX_TIMEOUT = 200; // leer cada 200ms

unsigned long proxCloseSince = 0;
bool buzzerForcedOff = false;
const unsigned long PROX_BUZZER_OFF_TIME = 3000; // tiempo mínimo proximidad para apagar el sistema


void beepShort() {
  tone(BUZZER_PIN, buzzerFreqHigh);
  delay(150);
  noTone(BUZZER_PIN);
}

void beepLong() {
  tone(BUZZER_PIN, buzzerFreqLow);
  delay(600);
  noTone(BUZZER_PIN);
}

void buzzerOn() {
  beepShort();
  delay(150);
  beepShort();
  buzzerState = true;
  Serial.println("Buzzer ENCENDIDO");
}

void buzzerOff() {
  beepLong();
  buzzerState = false;
  Serial.println("Buzzer APAGADO");
}

// ===================== WIFI =====================
void setup_wifi() {
  Serial.print("Conectando a WiFi ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectada");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  // Serial.print("Mensaje MQTT [");
  // Serial.print(topic);
  // Serial.print("]: ");
  // Serial.println(msg);

  String topic_str = String(topic);
  Serial.print(topic_str=="buzzer");

  // buzzer
  if (topic_str == "buzzer") {
    int valor = msg.toInt();
    if (valor == 1) {
      if (!buzzerState) {
        buzzerOn();      // 2 pitidos cortos
      }
    }
    else if (valor == 0) {
      if (buzzerState) {
        buzzerOff();     // 1 pitido largo
      }
    }
    else {
      Serial.println("Comando buzzer desconocido");
    }
  }

  // lcd
  else if (topic_str == "lcd") {
    Serial.print("Mensaje para LCD: ");
    Serial.println(msg);

    // lcd.clear();
    // lcd.print(msg);
  }

  else {
    Serial.println("Dispositivo desconocido");
  }
}

bool connectToBroker(const char* server, const char* name) {
  client.setServer(server, mqtt_port);
  Serial.print("Conectando MQTT (");
  Serial.print(name);
  Serial.print(")... ");

  if (client.connect("ESP32-BUZZER")) {
    Serial.println("OK");
    client.subscribe(topic_sub_cmd);
    return true;
  }

  Serial.println("FAILED");
  return false;
}



void reconnect() {
  // preferiblemente usar fog
  if (usingFog) {
    if (connectToBroker(mqtt_fog, "Fog")) {
      usingFog = true;
      return;
    }

    // fog no disponible -> conectar cloudlet
    if (connectToBroker(mqtt_cloud, "Cloudlet")) {
      usingFog = false;
      return;
    }
  }

  // si estamos en cloudlet...
  else {
    // reintentar cloudlet
    if (connectToBroker(mqtt_cloud, "Cloudlet")) {
      usingFog = false;
      return;
    }
  }
  delay(2000);
}


// interrupt
void IRAM_ATTR interruptRoutine() {
  isr_flag = true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  pinMode(WATERSENSOR_PIN, INPUT);

  pinMode(APDS9930_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(APDS9930_INT), interruptRoutine, FALLING);

  // ---- APDS INIT ----
  if (!apds.init()) {
    Serial.println("Error inicializando APDS9930");
    while (1);
  }
  apds.setProximityGain(PGAIN_2X);
  apds.setProximityIntLowThreshold(PROX_INT_LOW);
  apds.setProximityIntHighThreshold(PROX_INT_HIGH);
  apds.enableProximitySensor(true);
  Serial.println("APDS9930 listo");

#ifdef DUMP_REGS
  uint8_t reg, val;
  for (reg = 0x00; reg <= 0x19; reg++) {
    if (reg != 0x10 && reg != 0x11) {
      apds.wireReadDataByte(reg, val);
      Serial.print(reg, HEX);
      Serial.print(": 0x");
      Serial.println(val, HEX);
    }
  }
#endif

  setup_wifi();

  // client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  Serial.println("Sistema listo, esperando comandos MQTT");
}

// ===================== LOOP =====================
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // reconectar con fog o cloudlet
  if (!usingFog && client.connected()) {
    if (millis() - lastFogRetry > FOG_RETRY_INTERVAL) {
      lastFogRetry = millis();

      Serial.println("Intentando volver al Fog...");
      client.disconnect();

      if (connectToBroker(mqtt_fog, "Fog")) {
        usingFog = true;
        Serial.println("Reconectado al Fog");
      } else {
        connectToBroker(mqtt_cloud, "Cloudlet");
        usingFog = false;
      }
    }
  }

  unsigned long now = millis();
  if (now - lastSend >= interval) {
    lastSend = now;

    int waterLevel = analogRead(WATERSENSOR_PIN);

    // Crear mensaje JSON
    String payload = String(waterLevel);

    client.publish(topic_pub_cmd, payload.c_str());

    // Serial.print("Nivel de agua enviado: ");
    // Serial.println(payload);
  }

  if (isr_flag) {
    isr_flag = false;

    if (apds.readProximity(proximity_data)) {

      lastProxEvent = millis();

      bool newState = proxState;

      // decidir estado candidato usando histéresis
      if (proximity_data > PROX_ON_THRESHOLD) {
        newState = true;
      }
      else if (proximity_data < PROX_OFF_THRESHOLD) {
        newState = false;
      }

      // si el estado candidato cambia, arrancar temporizador
      if (newState != proxCandidateState) {
        proxCandidateState = newState;
        proxCandidateSince = millis();
      }

      // si se mantiene estable suficiente tiempo, aceptar cambio
      if ((millis() - proxCandidateSince >= PROX_STABLE_TIME) &&
          (proxState != proxCandidateState)) {

        proxState = proxCandidateState;

        if (proxState) {
          client.publish(topic_pub_prox, "1");
          Serial.println("MQTT proximidad -> 1 (estable)");
        } else {
          client.publish(topic_pub_prox, "0");
          Serial.println("MQTT proximidad -> 0 (estable)");
        }
      }

    } else {
      Serial.println("Error leyendo proximidad");
    }

    apds.clearProximityInt();
  }

  if (proxState && (millis() - lastProxEvent >= PROX_TIMEOUT)) {
    Serial.println("Timeout proximidad -> forzando 0");
    client.publish(topic_pub_prox, "0");
    proxState = false;
  }


  // si lleva más de PROX_BUZZER_OFF_TIME, mandar apagar el buzzer
  if (proxState) {
    // si acaba de entrar en proximidad, iniciar contador
    if (proxCloseSince == 0) {
      proxCloseSince = millis();
      buzzerForcedOff = false;
    }

    // si lleva suficiente tiempo cerca y no se ha ejecutado aún
    if (!buzzerForcedOff &&
        (millis() - proxCloseSince >= PROX_BUZZER_OFF_TIME)) {

      Serial.println("Proximidad prolongada -> apagando buzzer");

      // apagar buzzer local
      if (buzzerState) {
        buzzerOff();
      }

      // enviar MQTT
      client.publish("buzzer/cmd", "0");

      buzzerForcedOff = true;
    }
  }
  else {
    // no hay objeto proximo, reiniciar logica
    proxCloseSince = 0;
    buzzerForcedOff = false;
  }
}
