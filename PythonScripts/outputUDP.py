import socket
import time

# ESP32 AP credentials
ESP_IP = "192.168.4.1"  # Default IP for ESP32 AP mode
PORT = 5000  # Must match the receiver's port

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_data(payload, header):
    packet = bytes([header]) + bytes(payload)  # Create packet with header + payload
    sock.sendto(packet, (ESP_IP, PORT))
    print(f"Sent packet: {[hex(b) for b in packet]}")

# Main loop
try:
    while True:
        payload = [0x00, 0x13, 0xFF, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]  # Same payload as ESP32
        header = 0b00000111  # Header
        send_data(payload, header)
        time.sleep(2)  # Wait 2 seconds
except KeyboardInterrupt:
    print("Stopped by user")
    sock.close()
