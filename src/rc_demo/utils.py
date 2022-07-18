import rospy

from std_msgs.msg import String
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool


class ControlMessage:
    """Send control message via MavROS"""

    def __init__(self) -> None:
        self.control_pub = rospy.Publisher('/rc_demo', String, queue_size=1)

    def send_control(self, roll: float, pitch: float, yaw: float, throttle: float) -> None:
        """Send control message

        Args:
            roll: roll
            pitch: pitch
            yaw: yaw
            throttle: throttle
        """
        msg = String()
        msg.data = "{roll},{pitch},{yaw},{throttle}".format(roll=roll,pitch=pitch,yaw=yaw,throttle=throttle)
        self.control_pub.publish(msg)


class MAVROSCommander:
    def __init__(self):
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

    def set_mode(self, mode):
        offb_set_mode = SetMode()
        offb_set_mode.custom_mode = mode
        try:
            resp1 = self.set_mode_client(0, offb_set_mode.custom_mode)
            return resp1.mode_sent
        except:
            return False

    def set_arm(self, value):
        arm_cmd = CommandBool()
        arm_cmd.value = value
        try:
            arm_client_1 = self.arming_client(arm_cmd.value)
            return arm_client_1.success
        except:
            return False

