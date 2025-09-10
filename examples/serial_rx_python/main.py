import serial

# Change COM3 to your serial port (Linux: "/dev/ttyUSB0", Mac: "/dev/tty.SLAB_USBtoUART")
PORT = "COM4"
BAUD = 115200  # Default for ESP32 console

with serial.Serial(PORT, BAUD, timeout=1) as ser:
    print(f"Listening on {PORT} at {BAUD} baud...")
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print("Received:", line)
