# Author(s):
#   Daniel Opara <hello@danielopara.me>


import serial
import sqlite3

# Set up serial connection (adjust COM port as necessary)
ser = serial.Serial('COM3', 115200)

# Set up database connection
conn = sqlite3.connect('sensor_data.db')
c = conn.cursor()

# Create table (do this once)
c.execute('''CREATE TABLE IF NOT EXISTS data (timestamp DATETIME, temperature REAL, pressure REAL, altitude REAL)''')


""""
// Arduino code to write data to SD card
Data Format: time, temperature, altimeter_temp, pressure, altitude, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z

"""


# Read data from serial port and insert into database
while True:
    line = ser.readline()  # Read a line of data from the Serial port
    print(line.decode().strip())  # Print the line to the console

    try:
        # Assume data is comma-separated
        timestamp, temperature, pressure, altitude = line.decode().strip().split(',')
        
        # Insert data into database
        c.execute("INSERT INTO data VALUES (?, ?, ?, ?)", (timestamp, temperature, pressure, altitude))
        conn.commit()
    except Exception as e:
        print(f"Error processing line: {line}, Error: {e}")

# Don't forget to close the connection
ser.close()
conn.close()
