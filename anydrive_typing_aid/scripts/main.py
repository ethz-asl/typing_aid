#!/usr/bin/env python

import os
import rospy

from std_msgs.msg import String

from anydrive_typing_aid.utils.anydrive_interface import AnydriveInterface
from anydrive_typing_aid.controllers.position_controller import PositionController
from anydrive_typing_aid.controllers.torque_controller import TorqueController
from anydrive_typing_aid.controllers.impedance_controller import ImpedanceController
from anydrive_typing_aid.controllers.friction_controller import FrictionController
import anydrive_typing_aid.utils.utilities as utilities


if __name__ == "__main__":
    rospy.init_node("typing_aid")

    save_dir = os.path.expanduser("~/Data/TypingAid")
    utilities.create_dir(save_dir)

    drv_interface = AnydriveInterface()
    drv_interface.set_fsm_state(4)

    rate_hz = 150.0

    # ctrl = PositionController(drv_interface, rate_hz, save_dir)
    ctrl = TorqueController(drv_interface, rate_hz, save_dir)
    # ctrl = ImpedanceController(drv_interface, rate_hz, save_dir)
    # ctrl = FrictionController(drv_interface, rate_hz, save_dir)

    ctrl.plot_trajectory()

    rate = rospy.Rate(rate_hz)
    rospy.loginfo("Send first trigger to start...")
    rospy.wait_for_message("lift_arm", String)
    rospy.loginfo("Starting loop ")
    ctrl.subscribe()
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
