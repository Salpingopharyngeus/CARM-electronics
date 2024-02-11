

#include <SPI.h>
#include <RH_RF95.h>

// LoRa pins configuration for receiver
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// Frequency should match the transmitter's frequency
#define RF95_FREQ 433.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

struct SensorData {
    float time;
    float temperature;
    float altimeter_temp;
    float pressure;
    float altitude;
    float accel_x;
    float accel_y;
    float accel_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for Serial port to connect. Needed for native USB
    }
  
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    // Initialize LoRa radio
    if (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    }

    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }

    Serial.println("LoRa Receiver");
}

void loop() {
    if (rf95.available()) {
        // Buffer to hold the incoming message
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        // Receive a message into the buffer
        if (rf95.recv(buf, &len)) {
            // Successfully received a message
            Serial.println("Received message:");

            // Deserialize the data
            SensorData receivedData;
            memcpy(&receivedData, buf, sizeof(receivedData));

            // Print the received data to Serial monitor
            Serial.print("Time: ");
            Serial.println(receivedData.time);
            Serial.print("Temperature: ");
            Serial.println(receivedData.temperature);
            // Continue printing other fields...
            Serial.print("Altitude: ");
            Serial.println(receivedData.altitude);
            // Add other sensor data fields as needed

        } else {
            // Message received, but decoding failed
            Serial.println("Receive failed");
        }
    }
    // No delay needed, as we want to keep checking for messages continuously
}
