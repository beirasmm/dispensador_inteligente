#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// --- Configuración Wi-FI ---
const char* ssid     = "65E8";
const char* password = "pdecpk2kk6htan";

//--- Configuración de IPs (Punto 3 de la Práctica) ---
// Se asume IP fija para la Raspberry Pi como dispositivo Fog 
const char* ip_fog      = "172.29.111.151"; 192.168.1.130 //IP de Hotspot Movil
const char* ip_cloudlet = "192.168.1.130"; // IP del wifi casa
const int mqtt_port     = 1883;

// --- Pines ---
#define IR_SENSOR_PIN  4   
#define SERVO_PIN      18  

// Conf Servo
const int POS_CERRADO = 0;
const int POS_ABIERTO = 90;

// MQTT Topics
const char* topic_data = "devices/NAPIoT-P4-USC2/agua/estado";
const char* topic_cmd  = "devices/NAPIoT-P4-USC2/agua/cmd";

// --- Objetos ---
WiFiClient espClient;
PubSubClient client(espClient);
Servo miServo;

bool obstaculoDetectado = false;
bool estadoAnterior = false;

void setup_wifi() {
  delay(10);
  Serial.print("\nConectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Conectada.");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  if (msg == "OFF") miServo.write(POS_CERRADO);
}

// Función modificada para cumplir el punto 3 de la práctica 
void reconnect() {
  while (!client.connected()) {
    // 1. Intentar primero con el dispositivo Fog (Raspberry Pi) 
    Serial.print("Intentando conexión al FOG (Raspberry)...");
    client.setServer(ip_fog, mqtt_port);
    
    if (client.connect("ESP32_WaterControl")) {
      Serial.println("Conectado al FOG");
      client.subscribe(topic_cmd);
    } else {
      // 2. Si falla, intentar con el Cloudlet 
      Serial.print("FOG fallido (rc=");
      Serial.print(client.state());
      Serial.println("). Intentando CLOUDLET...");
      
      client.setServer(ip_cloudlet, mqtt_port);
      if (client.connect("ESP32_WaterControl")) {
        Serial.println("Conectado al CLOUDLET");
        client.subscribe(topic_cmd);
      } else {
        Serial.println("Error en ambos. Reintentando en 5s...");
        delay(5000);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(IR_SENSOR_PIN, INPUT);
  
  ESP32PWM::allocateTimer(0);
  miServo.setPeriodHertz(50); 
  miServo.attach(SERVO_PIN, 500, 2400);
  miServo.write(POS_CERRADO);

  setup_wifi();
  // Inicialmente apuntamos al Fog, la función reconnect se encarga del resto
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  obstaculoDetectado = (digitalRead(IR_SENSOR_PIN) == LOW);

  if (obstaculoDetectado != estadoAnterior) {
    char payload[64];
    if (obstaculoDetectado) {
      miServo.write(POS_ABIERTO);
      sprintf(payload, "{\"status\":\"OPEN\", \"msg\":\"Obstaculo detectado\"}");
    } else {
      miServo.write(POS_CERRADO);
      sprintf(payload, "{\"status\":\"CLOSED\", \"msg\":\"Camino despejado\"}");
    }
    client.publish(topic_data, payload);
    estadoAnterior = obstaculoDetectado;
  }
  delay(100); 
}
