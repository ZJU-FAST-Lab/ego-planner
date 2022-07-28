#!/usr/bin/env python3

import sys
import rospy

from mavros_msgs.msg import RCIn

from utils import ControlMessage

class RCHandler:
    """Receieve RC stick data."""

    def __init__(self) -> None:
        self.rc_sub = rospy.Subscriber("/mavros/rc/in", RCIn, self.callback_rc_in)
        self.control_msg = ControlMessage()

        self.is_calibrated = False
        self.calib_count = 0

        self.val_range = [9999, 0.0]

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.throttle = 0.0

        self.cmd_magnitude = 1.0


    def callback_rc_in(self, msg: RCIn) -> None:
        """Receieves /mavros/rc/in

        Receieved stick message will be sent
        """
        for i in range(4):
            self.val_range[0] = min(self.val_range[0], msg.channels[i])
            self.val_range[1] = max(self.val_range[1], msg.channels[i])

        sub_val = self.val_range[0]
        divider = (self.val_range[1] - self.val_range[0])

        if divider < 500:
            return

        # Normalize 0 ~ 1
        self.roll = (msg.channels[0] - sub_val) / divider
        self.pitch = ((msg.channels[1] - sub_val) / divider)
        self.throttle = (msg.channels[2] - sub_val) / divider
        self.yaw = (msg.channels[3] - sub_val) / divider

        # Normalize -1 ~ 1
        self.roll = (self.roll - 0.5) * 2
        self.pitch = (self.pitch - 0.5) * 2
        self.throttle = (self.throttle - 0.5) * 2
        self.yaw = (self.yaw - 0.5) * 2

        # Need to calibrate for the RC
        # TODO(jeikeilim): Find better way to calibrate RC
        if self.roll == 0.0 and self.pitch == 0.0 and not self.is_calibrated:
            self.calib_count += 1

            if self.calib_count > 10:
                self.is_calibrated = True
        else:
            self.calib_count = 0

        if not self.is_calibrated:
            return

        self.control_msg.send_control(-self.roll * self.cmd_magnitude,
                          -self.pitch * self.cmd_magnitude,
                          -self.yaw * self.cmd_magnitude,
                          self.throttle * self.cmd_magnitude)


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    rospy.init_node("flight_control_node", anonymous=True)

    rc_handler = RCHandler()

    rospy.spin()

