from gpiozero import Button
import time

# Initialize the switches
# pull_up=True turns on the Pi's internal resistor (expecting a Ground connection)
# bounce_time=0.1 ignores the tiny mechanical vibrations of the metal lever
left_switch = Button(9, pull_up=True, bounce_time=0.1)
center_switch = Button(11, pull_up=True, bounce_time=0.1)
right_switch = Button(10, pull_up=True, bounce_time=0.1)

# Define the callback functions
def left_click():
    print("🟢 LEFT switch (GPIO 9) clicked!")

def center_click():
    print("🟡 CENTER switch (GPIO 11) clicked!")

def right_click():
    print("🔵 RIGHT switch (GPIO 10) clicked!")

# Attach the functions to the hardware events
left_switch.when_pressed = left_click
center_switch.when_pressed = center_click
right_switch.when_pressed = right_click

print("🤖 Bumper Switch Diagnostics Online.")
print("Waiting for manual clicks... (Press Ctrl+C to exit)")

# Keep the script alive
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nDiagnostics terminated.")