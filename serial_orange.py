import serial
import time

serial_port = '/dev/ttyS0'
baud_rate = 115200

ser = serial.Serial(serial_port, baud_rate, timeout=1)

try:
    print("Serial port opened successfully")

    while True:
        try:
            #time.sleep(10)
            if ser.in_waiting > 0:
                data = ser.readline().rstrip(b'\n').decode('utf-8')
                #raw_data=ser.read(1)
                print("Arduino says:", data)
            #else:
                #print("Estoy esperando datos")
        except serial.SerialException as se:
            print(f"SerialException: {se}")
            #time.sleep(1)
        except KeyboardInterrupt:
            print("\nExiting the script.")
            time.sleep(1)
except serial.SerialException as se:
    print(f"SerialException: {se}")
    time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting the script.")
    time.sleep(1)
finally:
    ser.close()
