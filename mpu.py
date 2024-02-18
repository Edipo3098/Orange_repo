import smbus
import time

# Define the I2C bus and MPU9250 address
i2c_bus = 0  # On Orange Pi, I2C-0 is commonly used
mpu9250_address = 0x68  # MPU9250 default I2C address

# Create an smbus object
i2c_bus = 1  # Assuming you want to use /dev/i2c-1
bus = smbus.SMBus(i2c_bus)

# MPU9250 register addresses
MPU9250_WHO_AM_I = 0x75
MPU9250_ACCEL_XOUT_H = 0x3B
MPU9250_ACCEL_YOUT_H = 0x3D
MPU9250_ACCEL_ZOUT_H = 0x3F
MPU9250_GYRO_XOUT_H = 0x43
MPU9250_GYRO_YOUT_H = 0x45
MPU9250_GYRO_ZOUT_H = 0x47


accel_calibration = {
    'a_x': 0.99910324,
    'm': 0.05304036,
    'b': 0.0  # You can add the bias term if needed
}

gyro_calibration = {
    'a_x': 1.00009231,
    'm': 0.01032118,
    'b': 0.0  # You can add the bias term if needed
}

# Constants for sensitivity values
ACCEL_SENSITIVITY = 16384.0  # LSB/g for +/- 2g range
GYRO_SENSITIVITY = 131.0  # LSB/dps for +/- 250 dps range


# Check the WHO_AM_I register to verify communication
who_am_i = bus.read_byte_data(mpu9250_address, MPU9250_WHO_AM_I)
print(f"MPU9250 WHO_AM_I: {hex(who_am_i)}")


# Function to read sensor data
def read_sensor_data(register, calibration_params, sensitivity):
    high = bus.read_byte_data(mpu9250_address, register)
    low = bus.read_byte_data(mpu9250_address, register + 1)
    value = (high << 8) | low

    if value > 32767:
        value -= 65536

    # Apply calibration
    calibrated_value = calibration_params['a_x'] * value * calibration_params['m'] + calibration_params['b']
    
    # Convert to physical units
    physical_value = calibrated_value / sensitivity
    
    return physical_value


try:
    while True:
        # Read accelerometer data
        accel_x = read_sensor_data(MPU9250_ACCEL_XOUT_H, accel_calibration, ACCEL_SENSITIVITY)
        accel_y = read_sensor_data(MPU9250_ACCEL_YOUT_H, accel_calibration, ACCEL_SENSITIVITY)
        accel_z = read_sensor_data(MPU9250_ACCEL_ZOUT_H, accel_calibration, ACCEL_SENSITIVITY)

        # Read, calibrate, and convert gyroscope data to dps
        gyro_x = read_sensor_data(MPU9250_GYRO_XOUT_H, gyro_calibration, GYRO_SENSITIVITY)
        gyro_y = read_sensor_data(MPU9250_GYRO_YOUT_H, gyro_calibration, GYRO_SENSITIVITY)
        gyro_z = read_sensor_data(MPU9250_GYRO_ZOUT_H, gyro_calibration, GYRO_SENSITIVITY)

        # Print the sensor data
        print(f"Accel X: {accel_x}, Accel Y: {accel_y}, Accel Z: {accel_z}")
        print(f"Gyro X: {gyro_x}, Gyro Y: {gyro_y}, Gyro Z: {gyro_z}")

        # Pause for a short duration
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting the script.")
finally:
    # Close the I2C bus
    bus.close()