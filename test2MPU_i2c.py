
import smbus
import time

# Define the I2C bus numbers for MPU9250 and HMC5883L
mpu9250_bus = 1  # /dev/i2c-1
hmc5883l_bus = 2  # /dev/i2c-2

# Define the I2C addresses for MPU9250 and HMC5883L
mpu9250_address = 0x68  # MPU9250 default I2C address
hmc5883l_address = 0x1E  # HMC5883L I2C address

# Create SMBus objects for both buses
mpu9250_bus = smbus.SMBus(mpu9250_bus)  # I2C bus for MPU9250
hmc5883l_bus = smbus.SMBus(hmc5883l_bus)  # I2C bus for HMC5883L

# MPU9250 register addresses
MPU9250_WHO_AM_I = 0x75
MPU9250_ACCEL_XOUT_H = 0x3B
MPU9250_ACCEL_YOUT_H = 0x3D
MPU9250_ACCEL_ZOUT_H = 0x3F
MPU9250_GYRO_XOUT_H = 0x43
MPU9250_GYRO_YOUT_H = 0x45
MPU9250_GYRO_ZOUT_H = 0x47

# HMC5883L register addresses
HMC5883L_CONFIG_REG_A = 0x00
HMC5883L_CONFIG_REG_B = 0x01
HMC5883L_MODE_REG = 0x02
HMC5883L_DATA_XOUT_H = 0x03

# Calibration parameters
accel_calibration = {'a_x': 0.99910324, 'm': 0.05304036, 'b': 0.0}
gyro_calibration = {'a_x': 1.00009231, 'm': 0.01032118, 'b': 0.0}

# Constants for sensitivity values
ACCEL_SENSITIVITY = 16384.0  # LSB/g for +/- 2g range
GYRO_SENSITIVITY = 131.0  # LSB/dps for +/- 250 dps range
MAG_SENSITIVITY = 1090.0  # LSB/gauss (assumed for HMC5883L)

# Check the WHO_AM_I register to verify communication with MPU9250
who_am_i = mpu9250_bus.read_byte_data(mpu9250_address, MPU9250_WHO_AM_I)
print(f"MPU9250 WHO_AM_I: {hex(who_am_i)}")

# Initialize HMC5883L (configure registers)
def initialize_hmc5883l():
    # Write to configuration register A
    hmc5883l_bus.write_byte_data(hmc5883l_address, HMC5883L_CONFIG_REG_A, 0x70)  # 8-average, 15 Hz default, normal measurement
    # Write to configuration register B (Gain)
    hmc5883l_bus.write_byte_data(hmc5883l_address, HMC5883L_CONFIG_REG_B, 0x20)  # Gain = 1.3 Ga
    # Write to mode register (Continuous measurement mode)
    hmc5883l_bus.write_byte_data(hmc5883l_address, HMC5883L_MODE_REG, 0x00)  # Continuous measurement mode

# Read sensor data from MPU9250
def read_sensor_data(bus, register, calibration_params, sensitivity):
    high = bus.read_byte_data(mpu9250_address, register)
    low = bus.read_byte_data(mpu9250_address, register + 1)
    value = (high << 8) | low

    if value > 32767:
        value -= 65536

    # Apply calibration and convert to physical units
    calibrated_value = calibration_params['a_x'] * value * calibration_params['m'] + calibration_params['b']
    physical_value = calibrated_value / sensitivity
    return physical_value

# Read magnetometer data from HMC5883L
def read_magnetometer_data():
    mag_x_high = hmc5883l_bus.read_byte_data(hmc5883l_address, HMC5883L_DATA_XOUT_H)
    mag_x_low = hmc5883l_bus.read_byte_data(hmc5883l_address, HMC5883L_DATA_XOUT_H + 1)
    mag_z_high = hmc5883l_bus.read_byte_data(hmc5883l_address, HMC5883L_DATA_XOUT_H + 2)
    mag_z_low = hmc5883l_bus.read_byte_data(hmc5883l_address, HMC5883L_DATA_XOUT_H + 3)
    mag_y_high = hmc5883l_bus.read_byte_data(hmc5883l_address, HMC5883L_DATA_XOUT_H + 4)
    mag_y_low = hmc5883l_bus.read_byte_data(hmc5883l_address, HMC5883L_DATA_XOUT_H + 5)

    mag_x = (mag_x_high << 8) | mag_x_low
    mag_y = (mag_y_high << 8) | mag_y_low
    mag_z = (mag_z_high << 8) | mag_z_low

    # Handle negative values for signed data
    if mag_x > 32767:
        mag_x -= 65536
    if mag_y > 32767:
        mag_y -= 65536
    if mag_z > 32767:
        mag_z -= 65536

    # Convert raw values to Gauss
    mag_x = mag_x / MAG_SENSITIVITY
    mag_y = mag_y / MAG_SENSITIVITY
    mag_z = mag_z / MAG_SENSITIVITY

    return mag_x, mag_y, mag_z

try:
    initialize_hmc5883l()  # Initialize the magnetometer (HMC5883L)

    while True:
        # Read accelerometer data from MPU9250 (on I2C bus 1)
        accel_x = read_sensor_data(mpu9250_bus, MPU9250_ACCEL_XOUT_H, accel_calibration, ACCEL_SENSITIVITY)
        accel_y = read_sensor_data(mpu9250_bus, MPU9250_ACCEL_YOUT_H, accel_calibration, ACCEL_SENSITIVITY)
        accel_z = read_sensor_data(mpu9250_bus, MPU9250_ACCEL_ZOUT_H, accel_calibration, ACCEL_SENSITIVITY)

        # Read gyroscope data from MPU9250 (on I2C bus 1)
        gyro_x = read_sensor_data(mpu9250_bus, MPU9250_GYRO_XOUT_H, gyro_calibration, GYRO_SENSITIVITY)
        gyro_y = read_sensor_data(mpu9250_bus, MPU9250_GYRO_YOUT_H, gyro_calibration, GYRO_SENSITIVITY)
        gyro_z = read_sensor_data(mpu9250_bus, MPU9250_GYRO_ZOUT_H, gyro_calibration, GYRO_SENSITIVITY)

        # Read magnetometer data from HMC5883L (on I2C bus 2)
        mag_x, mag_y, mag_z = read_magnetometer_data()

        # Print MPU9250 sensor data (acceleration and gyro)
        print(f"Accel X: {accel_x}, Accel Y: {accel_y}, Accel Z: {accel_z}")
        print(f"Gyro X: {gyro_x}, Gyro Y: {gyro_y}, Gyro Z: {gyro_z}")

        # Print HMC5883L magnetometer data
        print(f"Mag X: {mag_x}, Mag Y: {mag_y}, Mag Z: {mag_z}")

        # Pause for a short duration
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting the script.")
finally:
    # Close the I2C buses
    mpu9250_bus.close()
    hmc5883l_bus.close()
