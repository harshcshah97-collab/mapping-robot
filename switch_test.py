#!/usr/bin/env python3
"""Interactive diagnostic for the three physical bumper switches."""

import time

from gpiozero import Button


def main():
    switches = {
        "LEFT": Button(9, pull_up=True, bounce_time=0.1),
        "CENTER": Button(11, pull_up=True, bounce_time=0.1),
        "RIGHT": Button(10, pull_up=True, bounce_time=0.1),
    }

    def reporter(name):
        return lambda: print(f"{name} bumper pressed")

    for name, switch in switches.items():
        switch.when_pressed = reporter(name)

    print("Bumper switch diagnostic online.")
    print("Press each switch. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Diagnostic terminated.")
    finally:
        for switch in switches.values():
            switch.close()


if __name__ == "__main__":
    main()
