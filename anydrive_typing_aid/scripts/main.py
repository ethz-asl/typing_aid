#!/usr/bin/env python

import os
import rospy

# from torque_profile import cte_mov
# from pid import pid
# from pos_control import pos_mov
# from imped_pos import Impedance_pos
# from imped_vel import impedance_vel
# from friction import Friction
# from basic_pos_control import Basic_controller

from anydrive_typing_aid.utils.anydrive_interface import AnydriveInterface
from anydrive_typing_aid.controllers.position_controller import PositionController
from anydrive_typing_aid.controllers.impedance_controller import ImpedanceController
import anydrive_typing_aid.utils.utilities as utilities


if __name__ == "__main__":
    rospy.init_node("typing_aid")

    save_dir = os.path.expanduser("~/Data/TypingAid")
    utilities.create_dir(save_dir)

    drv_interface = AnydriveInterface()
    drv_interface.set_fsm_state(4)

    rate_hz = 150.0

    controller = input(
        "Choose the controller method\n"
        "1 = position controller\n"
        "2 = torque controller\n"
        "3 = pid controller\n"
        "4 = impedance on pos controller\n"
        "5 = impedance on vel controller\n"
        "6 = friction controller\n"
    )
    if controller == 1:
        ctrl = PositionController(drv_interface, rate_hz, save_dir)
    # elif controller == 2:
    #     ctrl = cte_mov()
    # elif controller == 3:
    #     ctrl = pid()
    elif controller == 4:
        ctrl = ImpedanceController(drv_interface, rate_hz, save_dir)
    # elif controller == 5:
    #     ctrl = impedance_vel()
    # elif controller == 6:
    #     ctrl = Friction()
    else:
        rospy.logwarn("Invalid selection. Falling back to position controller.")
        ctrl = PositionController(drv_interface, rate_hz, save_dir)

    rate = rospy.Rate(rate_hz)
    rospy.loginfo("Starting loop ")
    try:
        while not rospy.is_shutdown():
            res = ctrl.step()
            if not res:
                break
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ran interrupt handler")
    finally:
        rospy.loginfo("Shutting down")
        ctrl.stop()
