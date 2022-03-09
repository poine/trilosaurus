#!/usr/bin/env python3

import time
import math
from trilobot import *
from trilobot import controller_mappings
from trilobot import simple_controller


# /usr/local/lib/python3.8/dist-packages/trilobot/
def create_mocute_controller(stick_deadzone_percent=0.1):
    """ Create a controller class for the MOCUTE Wireless controller.
    stick_deadzone_percent: the deadzone amount to apply to the controller's analog sticks
    """
    controller = simple_controller.SimpleController("MOCUTE-056_M06", exact_match=False)

    # Button and axis registrations for PS4 Controller
    # controller.register_button("Cross", 304, alt_name="A")
    # controller.register_button("Circle", 305, alt_name="B")
    # controller.register_button("Square", 308, alt_name="X")
    # controller.register_button("Triangle", 307, alt_name="Y")
    # controller.register_button("Options", 315, alt_name='Start')
    # controller.register_button("Share", 314, alt_name='Select')
    # controller.register_button("PS", 316, alt_name='Home')
    # controller.register_button("L1", 310, alt_name="LB")
    # controller.register_button("L2", 312, alt_name="LT")
    # controller.register_button("R1", 311, alt_name="RB")
    # controller.register_button("R2", 313, alt_name="RT")
    # controller.register_axis_as_button("Left", 16, -1, 0)
    # controller.register_axis_as_button("Right", 16, 1, 0)
    # controller.register_axis_as_button("Up", 17, -1, 0)
    # controller.register_axis_as_button("Down", 17, 1, 0)
    # controller.register_button("L3", 317, alt_name='LS')
    # controller.register_button("R3", 318, alt_name='RS')

    controller.register_axis("LX", 0, 0, 255, deadzone_percent=stick_deadzone_percent)
    controller.register_axis("LY", 1, 0, 255, deadzone_percent=stick_deadzone_percent)
    controller.register_axis("RX", 2, 0, 255, deadzone_percent=stick_deadzone_percent)
    controller.register_axis("RY", 3, 0, 255, deadzone_percent=stick_deadzone_percent)
    # controller.register_trigger_axis("L2", 4, 0, 255, alt_name="LT")
    # controller.register_trigger_axis("R2", 5, 0, 255, alt_name="RT")
    return controller



def main():
    print("Trilobot Remote Control Starting\n")
    RED, GREEN, BLUE = (255, 0, 0), (0, 255, 0), (0, 0, 255)
    tbot = Trilobot()

    controller = create_mocute_controller(stick_deadzone_percent=0.1)
    controller.connect()

    # Run an amination on the underlights to show a controller has been selected
    # for led in range(NUM_UNDERLIGHTS):
    #     tbot.clear_underlighting(show=False)
    #     tbot.set_underlight(led, RED)
    #     time.sleep(0.1)
    #     tbot.clear_underlighting(show=False)
    #     tbot.set_underlight(led, GREEN)
    #     time.sleep(0.1)
    #     tbot.clear_underlighting(show=False)
    #     tbot.set_underlight(led, BLUE)
    #     time.sleep(0.1)

    tbot.clear_underlighting()

    h = 0
    v = 0
    spacing = 1.0 / NUM_UNDERLIGHTS

    while True:

        if not controller.is_connected():
            controller.reconnect(10, True)
        try:
            # Get the latest information from the controller. This will throw a RuntimeError if the controller connection is lost
            controller.update(debug=False)
        except RuntimeError:
            # Lost contact with the controller, so disable the motors to stop Trilobot if it was moving
            tbot.disable_motors()

        if controller.is_connected():
            lx = controller.read_axis("RX")
            ly = 0 - controller.read_axis("LY")
            KL, KR = 0.8, 0.7
            tbot.set_left_speed(KL*ly  + KR*lx)
            tbot.set_right_speed(KL*ly - KR*lx)
        else:
            # Run a slow red pulsing animation to show there is no controller connected
            val = (math.sin(v) / 2.0) + 0.5
            tbot.fill_underlighting(val * 127, 0, 0)
            v += math.pi / 200

        time.sleep(0.01)

if __name__ == '__main__':
    main()
