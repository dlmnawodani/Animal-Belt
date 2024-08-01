 #include <FirebaseESP32.h>
#include <WiFi.h>
 #include <DHT.h>
#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define FIREBASE_HOST "your-project-id.firebaseio.com"
#define FIREBASE_AUTH "your-database-secret "

#define WIFI_SSID "your-SSID"
#define WIFI_PASSWORD "your-PASSWORD"

#define DHTPIN 4
#define DHTTYPE DHT11

#define RAIN_SENSOR_PIN 5  // The digital pin connected to the rain sensor

FirebaseData firebaseData;
DHT dht(DHTPIN, DHTTYPE);
MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(16, 17);  // GPS_RX_PIN, GPS_TX_PIN

int rainSensorPin = RAIN_SENSOR_PIN;
bool isRaining = false;

void setup() {
  Serial.begin(9600);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to WiFi!");

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  dht.begin();
  Wire.begin();
  mpu.initialize();

  gpsSerial.begin(9600);

  pinMode(rainSensorPin, INPUT);
}

void loop() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (!isnan(humidity) && !isnan(temperature)) {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%  Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");

    if (Firebase.setFloat(firebaseData, "/environment/humidity", humidity) && Firebase.setFloat(firebaseData, "/environment/temperature", temperature)) {
      Serial.println("Data sent to Firebase!");
    } else {
      Serial.println("Error sending data to Firebase");
      Serial.println(firebaseData.errorReason());
    }
  } else {
    Serial.println("Failed to read from DHT sensor!");
  }

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("Accelerometer: ");
  Serial.print("x = ");
  Serial.print(ax);
  Serial.print(", y = ");
  Serial.print(ay);
  Serial.print(", z = ");
  Serial.println(az);

  Serial.print("Gyroscope: ");
  Serial.print("x = ");
  Serial.print(gx);
  Serial.print(", y = ");
  Serial.print(gy);
  Serial.print(", z = ");
  Serial.println(gz);

  if (Firebase.setInt(firebaseData, "/sensors/accelerometer/x", ax) && Firebase.setInt(firebaseData, "/sensors/accelerometer/y", ay) && Firebase.setInt(firebaseData, "/sensors/accelerometer/z", az) && Firebase.setInt(firebaseData, "/sensors/gyroscope/x", gx) && Firebase.setInt(firebaseData, "/sensors/gyroscope/y", gy) && Firebase.setInt(firebaseData, "/sensors/gyroscope/z", gz)) {
    Serial.println("Sensor data sent to Firebase!");
  } else {
    Serial.println("Error sending sensor data to Firebase");
    Serial.println(firebaseData.errorReason());
  }

  isRaining = digitalRead(rainSensorPin);  // Read the state of the rain sensor

  if (!isRaining) {
    Serial.println("It's raining!");
    // Send the rain sensor data to Firebase
    if (Firebase.setBool(firebaseData, "/sensors/rain", true)) {
      Serial.println("Rain data sent to Firebase!");
    } else {
      Serial.println("Error sending rain data to Firebase");
      Serial.println(firebaseData.errorReason());
    }
  } else {
    Serial.println("No rain detected.");
    // Send the rain sensor data to Firebase
    if (Firebase.setBool(firebaseData, "/sensors/rain", false)) {
      Serial.println("Rain data sent to Firebase!");
    } else {
      Serial.println("Error sending rain data to Firebase");
      Serial.println(firebaseData.errorReason());
    }
  }
 
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();

        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);

        if (Firebase.setFloat(firebaseData, "/location/latitude", gps.location.lat()) && Firebase.setFloat(firebaseData, "/location/longitude", gps.location.lng())) {
          Serial.println("GPS data sent to Firebase!");
        } else {
          Serial.println("Error sending GPS data to Firebase");
          Serial.println(firebaseData.errorReason());
        }
      }
    }
  }

   delay(5000);
}
