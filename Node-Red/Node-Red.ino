#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#define echoPin 33 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 32 //attach pin D3 Arduino to pin Trig of HC-SR04

// Add your required sensor/component libraries here
// --
#include <MPU6050_tockn.h>
#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04

MPU6050 mpu6050(Wire);
int x = 0;
int leftMotor_speed, rightMotor_speed, servoAngle;
float distance = 0;
float angle;
int angle_X,angle_Y,angle_Z;
float temperature;

// Replace the next variables with your SSID/Password combination
const char* ssid = "LiToScoMi"; //CHANGE ME
const char* password = "Bench10Supreme"; //CHANGE ME 

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144"; 
const char* mqtt_server = "192.168.137.42"; //CHANGE ME

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

float distance = 0;

float calcXAngle()
{
 mpu6050.update();
  Serial.print("\nangleX : ");
  Serial.print(mpu6050.getAngleX());
  angle_X = (mpu6050.getAngleX());
  delay(200);
  return angle_X;
}

float calcYAngle()
{
 mpu6050.update();
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  angle_Y = (mpu6050.getAngleY());
  delay(200);
  return angle_Y;
}

float calcZAngle()
{
 mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.print(mpu6050.getAngleZ());
  angle_Z = (mpu6050.getAngleZ());
  delay(200);
  return angle_Z;
}

float CalcDistance(){
  float duration;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  delay(100);
  //Serial.print(distance);

  return distance;
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); 
  //Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}
 
void Transmission(int leftMotor_speed, int rightMotor_speed, int servoAngle) {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write((byte)(leftMotor_speed & 0x0000FF00) >> 8);
  Wire.write((byte)(leftMotor_speed & 0x000000FF));
  Wire.write((byte)(rightMotor_speed & 0x0000FF00) >> 8);
  Wire.write((byte)(rightMotor_speed & 0x000000FF));
  Wire.write((byte)(servoAngle & 0x0000FF00) >> 8);
  Wire.write((byte)(servoAngle & 0x000000FF));
  Wire.endTransmission();
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
    }
    else if(messageTemp == "off"){
      Serial.println("off");
    }
  }
      if (String(topic) == "esp32/motorSpeed") {
      int newSpeed = messageTemp.toInt();
      leftMotor_speed = newSpeed;
      rightMotor_speed = newSpeed;
      Transmission(leftMotor_speed,rightMotor_speed,servoAngle);
  }
  if (String(topic) == "esp32/servoAngle") {
      servoAngle = messageTemp.toInt();
      Transmission(leftMotor_speed,rightMotor_speed,servoAngle);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");

       client.subscribe("esp32/output");
       client.subscribe("esp32/servoAngle");
       client.subscribe("esp32/motorSpeed");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    // Add your own code here i.e. sensor measurements, publish topics & subscribe topics for GPIO control
    // --

    // --
    
    angleX = calcXAngle();

    char angleXString[8];
    dtostrf(angleX, 1, 2, angleXString);
    Serial.print("angleXc19: ");
    Serial.println(angleXString);
    client.publish("c19angleX", angleXString);

    delay(100);
    angleY = calcYAngle();

    char angleYString[8];
    dtostrf(angleY, 1, 2, angleYString);
    Serial.print("angleYc19: ");
    Serial.println(angleYString);
    client.publish("c19angleY", angleYString);

   delay(100);

    angleZ = calcZAngle();
    
    char angleZString[8];
    dtostrf(angleZ, 1, 2, angleZString);
    Serial.print("angleZc19: ");
    Serial.println(angleZString);
    client.publish("c19angleZ", angleZString);
 
   delay(100);
   
    distance = CalcDistance();
    char distString[8];
    dtostrf(distance, 1, 2, distString);
    Serial.print("distancec19: ");
    Serial.println(distString);
    client.publish("c19distance", distString);
   
  }
}
