import os
import pygame
import socket
import time

# ESP32 AP credentials
ESP_IP = "192.168.4.1"  # Default IP for ESP32 AP mode
PORT = 5000  # Must match the receiver's port

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Function to clear terminal output
def clear_terminal():
    os.system('cls' if os.name == 'nt' else 'clear')

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected!")
    pygame.quit()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Controller detected: {joystick.get_name()}")

# Function to normalize joystick values (-1 to 1) into 0-255
def normalize_axis(value):
    return int((value + 1) * 128)  # Convert -1 to 1 into 0 to 255

# Function to send UDP packets
def send_data(payload, header):
    packet = bytes([header]) + bytes(payload)  # Create packet with header + payload
    
    # Convert header to binary and payload to hex
    header_bin = f"{header:08b}"  # 8-bit binary format
    payload_hex = [f"{b:#04x}" for b in payload]  # Hex format for payload

    # Print formatted packet
    print(f"Sent packet: [{header_bin}] " + " ".join(payload_hex))

    # Send the packet over UDP
    sock.sendto(packet, (ESP_IP, PORT))

# Main loop
try:
    heartbeat = True
    heartbeatTime = time.time()
    sendTime = 0
    while True:
        pygame.event.pump()
        clear_terminal()
        print(time.time() - sendTime)
        sendTime = time.time()
        
        # Read joystick axes
        left_x = normalize_axis(joystick.get_axis(0))  # Left Stick X
        left_y = normalize_axis(joystick.get_axis(1))  # Left Stick Y
        right_x = normalize_axis(joystick.get_axis(2))  # Right Stick X
        right_y = normalize_axis(joystick.get_axis(3))  # Right Stick Y

        # Read button states
        button_A = joystick.get_button(0)
        button_B = joystick.get_button(1)  
        button_X = joystick.get_button(2)  
        button_Y = joystick.get_button(3) 
        button_LB = joystick.get_button(4)  
        button_RB = joystick.get_button(5)
        button_Back = joystick.get_button(6)
        button_Start = joystick.get_button(7)
        button_Xbox = joystick.get_button(8)
        button_LeftStick = joystick.get_button(9)
        button_RightStick = joystick.get_button(10)

        #Get the heartbeat
        if(time.time() - heartbeatTime > .25):
            heartbeatTime = time.time()
            heartbeat = not heartbeat


        # Print joystick and button values
        print(f"Left Stick: X={left_x}, Y={left_y}")
        print(f"Right Stick: X={right_x}, Y={right_y}")
        print(f"Button 1 (A): {button_A}, Button 2 (B): {button_B}")

        # Modify the header based on button states
        header = 0b10000000  # Base header
        if button_A:
            header |= 0b00000001  # Set bit 3 if Button 1 is pressed
        if button_B:
            header |= 0b00000010  # Set bit 2 if Button 2 is pressed    
        if heartbeat:
            header |= 0b00000100

        # Create payload with joystick values at first 4 bytes
        payload = [left_x, left_y, right_x, right_y] + [0] * 28  # 32-byte payload

        # Send the packet over UDP
        send_data(payload, header)

        # Wait to avoid flooding packets
        time.sleep(.05)

except KeyboardInterrupt:
    print("Stopped by user")
    sock.close()
    pygame.quit()
