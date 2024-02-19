import serial
import time

serial_port = '/dev/ttyS5'
baud_rate = 115200

ser = serial.Serial(serial_port, baud_rate, timeout=1)


try:
    # Open the serial port
   

    while True:
        try:
            # Send data over the serial connection
            data_to_send = "Hello, Arduino!"
            ser.write(data_to_send.encode())  # Encode string as bytes before sending

            # Wait for a moment
            time.sleep(5)

            # Read response from the serial connection
            received_data = ser.readline().decode().strip()
            print("Received:", received_data)

        except KeyboardInterrupt:
            # If Ctrl+C is pressed, break out of the loop
            print("Keyboard interrupt detected. Exiting...")
            break

finally:
    # Close the serial port, even if an exception occurs
    ser.close()
