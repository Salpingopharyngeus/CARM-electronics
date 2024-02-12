/**************************************************************
 *
 *                     sensor_setup.h
 *
 *     Author(s):  Daniel Opara,
 *     Date:       1/6/2023
 *
 *     Overview: Contains functions for setting up configurations for each sensor
 *                  connected to the Feather M0
 *
 *
 **************************************************************/

int MAX_ATTEMPTS = 20;



/*
 * setupSensorIMU
 * Parameters: Reference (pointer) to the instantiated IMU sensor object
 * Purpose: Sets up the range at which the sensor's individual components measure
 *           higher ranges = less precision but can measure larger movements
 * Returns: Bool representing whether or not sensor was set up
 * Notes: The sensor was set up in I2C. For more info, read the device documentation:
 *            https://learn.adafruit.com/adafruit-lsm9ds1-accelerometer-plus-gyro-plus-magnetometer-9-dof-breakout/overview
 */
bool setupSensorIMU(Adafruit_LSM9DS1 &lsm_obj) {
    Serial.println("Setting up IMU...");
    for (int attempts = 0; attempts < MAX_ATTEMPTS; attempts++) {
        // Attempt to begin communication with the sensor
        if (!lsm_obj.begin()) {
            Serial.println("Failed to communicate with LSM9DS1. Attempting again...");
            delay(2000); // Wait before retrying
        } else {
            lsm_obj.setupAccel(lsm_obj.LSM9DS1_ACCELRANGE_2G);
            lsm_obj.setupMag(lsm_obj.LSM9DS1_MAGGAIN_4GAUSS);
            lsm_obj.setupGyro(lsm_obj.LSM9DS1_GYROSCALE_245DPS);
            Serial.println("Complete!");
            return true; // Successfully set up the sensor
        }
    }
    return false; // Failed to set up the sensor after MAX_ATTEMPTS
}

/*
 * setupSensorBMP
 * Parameters: Reference (pointer) to the instantiated barometric pressure sensor object
 * Purpose: Configures various settings of the BMP3 sensor to control the oversampling of temperature and pressure,
 *          set the IIR (Infinite Impulse Response) filter coefficient, and define the output data rate
 * Returns: Bool representing whether or not sensor was set up
 * Notes: The sensor was set up in I2C. For more info, read the device documentation:
 *            https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx/overview
 */
bool setupSensorBMP(Adafruit_BMP3XX &bmp_obj) {
    Serial.println("Setting up barometric pressure sensor...");
    for (int attempts = 0; attempts < MAX_ATTEMPTS; attempts++) {
        if (!bmp_obj.begin_I2C()) {
            Serial.println("Could not find a valid BMP3 sensor, check wiring!");
            delay(2000); // Wait before retrying
        } else {
            // Set up oversampling and filter initialization
            bmp_obj.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
            bmp_obj.setPressureOversampling(BMP3_OVERSAMPLING_4X);
            bmp_obj.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
            bmp_obj.setOutputDataRate(BMP3_ODR_50_HZ);
            Serial.println("Complete!");
            return true; // Successfully set up the sensor
        }
    }
    return false; // Failed to set up the sensor after MAX_ATTEMPTS
}

/*
 * setupSensorTemp
 * Parameters: Reference (pointer) to the instantiated temp sensor object
 * Purpose: Connects the sensor to its address and sets the precision of measurements
 * Returns: Bool representing whether or not sensor was set up
 * Notes: The sensor was set up in I2C. The resolution of measurements impacts the time it takes to collect sample data,
 *            recommended to check out test/feather_scripts and view the comments on the example if you want to better understand how it works.
 *            For more info, read the device documentation:
 *            https://learn.adafruit.com/adafruit-mcp9808-precision-i2c-temperature-sensor-guide/overview
 */
bool setupSensorTemp(Adafruit_MCP9808 &tempsensor_obj) {
    Serial.println("Setting up temperature sensor...");
    for (int attempts = 0; attempts < MAX_ATTEMPTS; attempts++) {
        if (!tempsensor_obj.begin(0x18)) {
            Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
            delay(2000); // Wait before retrying
        } else {
            tempsensor_obj.setResolution(3); // Highest resolution
            Serial.println("Complete!");
            return true; // Successfully set up the sensor
        }
    }
    return false; // Failed to set up the sensor after MAX_ATTEMPTS
}