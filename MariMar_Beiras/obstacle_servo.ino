#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// --- Configuración Wi-FI ---
const char* ssid = "ssi";
const char* password = "psw";

//--- Configuración Mosquitto---
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;

// --- Pines -
#define IR_SENSOR_PIN  4   
#define SERVO_PIN      18   // Pin de señal para el Servo SG90


// Conf Servo
const int POS_CERRADO = 0;  // Válvula cerrada
const int POS_ABIERTO = 90; // Válvula abierta

// MQTT Topics
const char* topic_data = "devices/NAPIoT-P2-USC2/agua/estado";
const char* topic_cmd  = "devices/NAPIoT-P2-USC2/agua/cmd";

// --- Objetos ---
WiFiClient espClient;
PubSubClient client(espClient);
Servo miServo;

bool obstaculoDetectado = false;
bool estadoAnterior = false;

// --- Funciones de Conexión ---
void setup_wifi() {
  delay(10);
  Serial.print("\nConectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Conectada. IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  
  Serial.println("Mensaje MQTT recibido: " + msg);
  // Comando remoto para forzar cierre si fuera necesario
  if (msg == "OFF") {
    miServo.write(POS_CERRADO);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32_WaterControl")) {
      Serial.println("Conectado");
      client.subscribe(topic_cmd);
    } else {
      Serial.print("Error, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Configuración de Servo
  ESP32PWM::allocateTimer(0);
  miServo.setPeriodHertz(50); 
  miServo.attach(SERVO_PIN, 500, 2400);
  miServo.write(POS_CERRADO); // Iniciar cerrado

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  
  obstaculoDetectado = (digitalRead(IR_SENSOR_PIN) == LOW);

  if (obstaculoDetectado != estadoAnterior) {
    char payload[64];
    
    if (obstaculoDetectado) {
      // Abrir paso de agua
      miServo.write(POS_ABIERTO);
      digitalWrite(LED_BUILTIN, HIGH);
      sprintf(payload, "{\"status\":\"OPEN\", \"msg\":\"Obstaculo detectado\"}");
      Serial.println("Agua: ABIERTA");
    } else {
      // Cerrar paso de agua
      miServo.write(POS_CERRADO);
      digitalWrite(LED_BUILTIN, LOW);
      sprintf(payload, "{\"status\":\"CLOSED\", \"msg\":\"Camino despejado\"}");
      Serial.println("Agua: CERRADA");
    }

    client.publish(topic_data, payload);
    estadoAnterior = obstaculoDetectado;
  }
  
  delay(100); 
}
