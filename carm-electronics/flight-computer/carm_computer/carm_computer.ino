

/**************************************************************
 *
 *                     carm_computer.ino
 *
 *     Author(s):  Daniel Opara, Kenny Chua
 *     Date:       2/4/2024
 *
 *     Overview: Driver code for the rocket computer. Gets readings from the sensors
 *                  connected to it and transmits the data to the ground station.
 *
 *
 **************************************************************/


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h> // IMU module
#include "Adafruit_BMP3XX.h"  // BMP module
#include "Adafruit_MCP9808.h" // Temp sensor module
#include <Arduino_BuiltIn.h>
#include "sensor_setup.h"
#include <SD.h> // SD card module
#include <SPI.h> // SPI module
#include <RH_RF95.h> // Radio module


// - - - - - - - - - - - - - - - - - - - - - - - - - - -
//   Defining digital pins, constants, and sensor objects
// - - - - - - - - - - - - - - - - - - - - - - - - - - -
// BMP_SCK == SCL
#define BMP_SCK 18
// BMP_CS == CS
#define BMP_CS 17
#define SEALEVELPRESSURE_HPA (1013.25)
const int chipSelect = 22; // SD card module

// - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  LoRa RFM95 Configuration
// - - - - - - - - - - - - - - - - - - - - - - - - - - -

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define RF95_FREQ 433.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


// - - - - - - - - - - - - - - - - - - - - - - - - - - -
//   Defining Rocket States
// - - - - - - - - - - - - - - - - - - - - - - - - - - -

// NOT TRANSMITTING MUCH FOR THESE GUYS
// RECIEVING POWER
bool POWER_ON;
// SENSOR VALUES HAVE STABILIZED
bool LAUNCH_READY;

// WE ARE ACTUALLY TRANSMITTING HERE
// INDICATORS FROM SENSOR TELL US WE ARE IN THIS MODE
bool LAUNCH_MODE;
// TRANSITION FROM BURNOUT TO COAST PHASE IS WHEN JERK == 0 AND ACCEL IS NEGATIVE
bool POWERED_FLIGHT_PHASE;
bool COAST_PHASE;

bool APOGEE_PHASE; // determined by barometric altitude
bool DROGUE_DEPLOYMENT;
bool MAIN_DEPLOYMENT;
bool RECOVERY_PHASE;




Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_BMP3XX bmp;
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

File SD_File;

struct SensorData
{
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


void setup()
{
    Serial.begin(115200);
    setupSensorIMU(lsm);
    setupSensorBMP(bmp);
    setupSensorTemp(tempsensor);
    tempsensor.wake();
    
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //   Radio Initialziation
    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    
    // Initialising the radio module
    if (!rf95.init())
    {
        Serial.println("LoRa radio init failed");
        while (1);
    }

    if (!rf95.setFrequency(RF95_FREQ))
    {
        Serial.println("setFrequency failed");
        while (1);
    }

    //Sets the transmit power to 23 dBm
    rf95.setTxPower(23, false);

    Serial.println("LoRa radio init OK!");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //   SD Card Initialziation
    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    //Initialising the SD card
    Serial.print("Initializing SD card...");
    pinMode(chipSelect, OUTPUT);
    if (!SD.begin(chipSelect))
    {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialization done.");
  }

void loop()
{
    // get readings from all the sensors
    lsm.read();
    sensors_event_t a, m, g, temp;
    if (!bmp.performReading())
    {
        Serial.println("Failed to perform reading :(");
        return;
    }

    // create a new SensorData object
    SensorData data;
    data.time = millis() / 1000;



    // print out readings
    Serial.print("Temperature Sensor = ");
    Serial.println(tempsensor.readTempC());
    data.temperature = tempsensor.readTempC();

    Serial.print("Temperature = ");
    Serial.print(bmp.temperature);
    Serial.println(" *C");
    data.altimeter_temp = bmp.temperature;
    

    Serial.print("Pressure = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" kPa");
    data.pressure = bmp.pressure / 100.0;

    Serial.print("Approx. Altitude = ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    data.altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    Serial.println();
    lsm.getEvent(&a, &m, &g, &temp);
    Serial.print("Accel X: ");
    Serial.print(a.acceleration.x);
    data.accel_x = a.acceleration.x;


    Serial.print(" Y: ");
    Serial.print(a.acceleration.y);
    data.accel_y = a.acceleration.y;

    Serial.print(" Z: ");
    Serial.println(a.acceleration.z);
    data.accel_z = a.acceleration.z;

    Serial.print("Mag X: ");
    Serial.print(m.magnetic.x);
    data.mag_x = m.magnetic.x;

    Serial.print(" Y: ");
    Serial.print(m.magnetic.y);
    data.mag_y = m.magnetic.y;

    Serial.print(" Z: ");
    Serial.println(m.magnetic.z);
    data.mag_z = m.magnetic.z;

    Serial.print("Gyro X: ");
    Serial.print(g.gyro.x);
    data.gyro_x = g.gyro.x;

    Serial.print(" Y: ");
    Serial.print(g.gyro.y);
    data.gyro_y = g.gyro.y;

    Serial.print(" Z: ");
    Serial.println(g.gyro.z);
    data.gyro_z = g.gyro.z;

    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.print(data.time);
        dataFile.print(",");
        dataFile.print(data.temperature);
        dataFile.print(",");
        dataFile.print(data.altimeter_temp);
        dataFile.print(",");
        dataFile.print(data.pressure);
        dataFile.print(",");
        dataFile.print(data.altitude);
        dataFile.print(",");
        dataFile.print(data.accel_x);
        dataFile.print(",");
        dataFile.print(data.accel_y);
        dataFile.print(",");
        dataFile.print(data.accel_z);
        dataFile.print(",");
        dataFile.print(data.mag_x);
        dataFile.print(",");
        dataFile.print(data.mag_y);
        dataFile.print(",");
        dataFile.print(data.mag_z);
        dataFile.print(",");
        dataFile.print(data.gyro_x);
        dataFile.print(",");
        dataFile.print(data.gyro_y);
        dataFile.print(",");
        dataFile.println(data.gyro_z);
        dataFile.close();
        
        Serial.println("Data written to SD card");
    }
    else {
        Serial.println("error opening datalog.csv");
    }

    // Transmitting the data
    Serial.println("Sending packet: ");
    uint8_t buf[sizeof(data)];
    memcpy(buf, &data, sizeof(data));
    rf95.send(buf, sizeof(buf));
    rf95.waitPacketSent();
    Serial.println("Sent a packet");

    //Sets the transmision rate to 1 second.
    delay(1000);
 



}
