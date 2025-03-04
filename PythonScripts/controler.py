import os
import pygame

def clear_terminal():
    os.system('cls' if os.name == 'nt' else 'clear')

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected!")
    pygame.quit()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

while True:
    pygame.event.pump()
    clear_terminal()  # Clear the screen

    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    dpad = joystick.get_hat(0)

    print(f"Buttons: {buttons}")
    print(f"Axes: {axes}")
    print(f"D-Pad: {dpad}")

    if buttons[6]:  # Exit if "Back" button is pressed
        break

    pygame.time.wait(100)

pygame.quit()
