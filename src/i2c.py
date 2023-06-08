import smbus

bus = smbus.SMBus(1)  # Use I2C bus 1

left = 0x08  # Address of the Arduino
right = 0x09

def receive_data():
    received_left = bus.read_byte(left)
    received_right = bus.read_byte(right)
    print("Received Data:", received_left, received_right)

def send_data(data):
    bus.write_byte(left, data)
    bus.write_byte(right, data)

while True:
    bus.write_quick(left)
    bus.write_quick(right)

    receive_data()

    data_to_send = 200
    send_data(data_to_send)
