#include <LiquidCrystal_I2C.h>
#include <AltSoftSerial.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Bluetooth module setup
#define bluetoothRxPin 2  // Define your Bluetooth module RX pin
#define bluetoothTxPin 3  // Define your Bluetooth module TX pin
SoftwareSerial bluetooth(bluetoothRxPin, bluetoothTxPin);

// Emergency phone number with country code
const String EMERGENCY_PHONE = "+919798461659";

// GSM module setup
#define btrxPin 2
#define bttxPin 3
SoftwareSerial bt(btrxPin, bttxPin);

// GPS module setup
#define gpsRxPin 9
#define gpsTxPin 8
AltSoftSerial neogps;
TinyGPSPlus gps; // Define gps object globally

// Variables for storing GPS data
String latitude, longitude;

// Hardware setup
#define BUZZER 12
#define BUTTON 11
#define xPin A1
#define yPin A2
#define zPin A3

// Variables for impact detection
byte updateflag;
int xaxis = 0, yaxis = 0, zaxis = 0;
int deltx = 0, delty = 0, deltz = 0;
int vibration = 2, devibrate = 75;
int magnitude = 0;
int sensitivity = 150;
boolean impact_detected = false;
unsigned long time1;
unsigned long impact_time;
unsigned long alert_delay = 10000; // 10 seconds

/*****************************************************************************************
 * setup() function
 *****************************************************************************************/
void setup() {
    // Initialize LCD
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Crash Detect");
    lcd.setCursor(0, 1);
    lcd.print("System");

    // Serial communication
    Serial.begin(9600);
    bt.begin(9600);
    neogps.begin(9600);
    bluetooth.begin(9600); // Initialize Bluetooth serial communication

    // Pin mode setup
    pinMode(BUZZER, OUTPUT);
    pinMode(BUTTON, INPUT_PULLUP);

    // Initialize time
    time1 = micros();

    // Read calibrated values for impact detection
    xaxis = analogRead(xPin);
    yaxis = analogRead(yPin);
    zaxis = analogRead(zPin);
}

/*****************************************************************************************
 * loop() function
 *****************************************************************************************/
void loop() {
    // Call impact routine every 2ms
    if (micros() - time1 > 1999) {
        Impact();
    }

    // If impact detected
    if (updateflag > 0) {
        updateflag = 0;
        Serial.println("Impact detected!!");
        Serial.print("Magnitude:");
        Serial.println(magnitude);

        // Get GPS data
        getGps();

        // Sound buzzer
        digitalWrite(BUZZER, HIGH);
        impact_detected = true;
        impact_time = millis();

        // Display crash information on LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Crash Detected");
        lcd.setCursor(0, 1);
        lcd.print("Magnitude:" + String(magnitude));
        lcd.setCursor(0, 0);
        delay(2000);
        lcd.clear();
        lcd.print("Call Number");
        lcd.setCursor(0, 1);
        lcd.print(EMERGENCY_PHONE);

        // Send GPS location via Bluetooth
        String message = "GPS Location Data\rhttp://maps.google.com/maps?q=loc:" + latitude + "," + longitude;
        sendBluetoothMessage(message);
    }

    // Check if it's time to send alert
    if (impact_detected && (millis() - impact_time >= alert_delay)) {
        digitalWrite(BUZZER, LOW);
        sendAlert();
        impact_detected = false;
        impact_time = 0;
    }

    // Reset impact detection if button pressed
    if (digitalRead(BUTTON) == LOW) {
        delay(200);
        digitalWrite(BUZZER, LOW);
        impact_detected = false;
        impact_time = 0;
    }
}

/*****************************************************************************************
 * Impact() function
 *****************************************************************************************/
void Impact() {
    time1 = micros(); // resets time value

    int oldx = xaxis;
    int oldy = yaxis;
    int oldz = zaxis;

    xaxis = analogRead(xPin);
    yaxis = analogRead(yPin);
    zaxis = analogRead(zPin);

    vibration--;
    if (vibration < 0) vibration = 0;

    if (vibration > 0) return;

    deltx = xaxis - oldx;
    delty = yaxis - oldy;
    deltz = zaxis - oldz;

    magnitude = sqrt(sq(deltx) + sq(delty) + sq(deltz));

    if (magnitude >= sensitivity) {
        updateflag = 1;
        vibration = devibrate;
    } else {
        magnitude = 0;
    }
}

/*****************************************************************************************
 * getGps() Function
 *****************************************************************************************/
void getGps() {
    boolean newData = false;
    for (unsigned long start = millis(); millis() - start < 2000;) {
        while (neogps.available()) {
            if (gps.encode(neogps.read())) {
                newData = true;
                break;
            }
        }
    }

    if (newData) {
        latitude = String(gps.location.lat(), 6);
        longitude = String(gps.location.lng(), 6);
        newData = false;
    } else {
        Serial.println("No GPS data is available");
        latitude = "";
        longitude = "";
    }

    Serial.print("Latitude= ");
    Serial.println(latitude);
    Serial.print("Longitude= ");
    Serial.println(longitude);
}

/*****************************************************************************************
 * sendAlert() function
 *****************************************************************************************/
void sendAlert() {
    String sms_data = "Accident Alert!!\rhttp://maps.google.com/maps?q=loc:" + latitude + "," + longitude;
    sendBluetoothMessage(sms_data); // Send the alert message over Bluetooth
}

/*****************************************************************************************
 * sendBluetoothMessage() function
 *****************************************************************************************/
void sendBluetoothMessage(String message) {
    bluetooth.println(message);
}
