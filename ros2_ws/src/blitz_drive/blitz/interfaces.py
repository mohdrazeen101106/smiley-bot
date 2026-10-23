from blitz import Blitz
from robot_interfaces.msg import BnoReading
from robot_interfaces.msg import Velocity, Encoder
from robot_interfaces.msg import SpearheadgripperTOF

blitz_interfaces = {str : Blitz}

# ============================================
# DRIVE ESP INTERFACES (mcu_pio)
# BNO sensor + drive motors + VL53L0X ToF
# ============================================
blitz_interfaces = {

    "BnoReading" : Blitz(
        topic="current_angle",
        msg_id=1,
        struct_fmt="fff",
        fields=["current_angle","pitch","roll"],
        ros_msg=BnoReading,
        from_mcu=True
    ),

    "Spearhead_gripper_TOF" : Blitz(
        topic="spearheadgripperTOF", msg_id=26, struct_fmt="ff",
        fields=["sensor1","sensor2"], ros_msg=SpearheadgripperTOF, from_mcu=True
    ),

    "Velocity" : Blitz(
        topic="cmd_vel",
        msg_id=5,
        struct_fmt="fff",
        fields=["vx","vy","vw"],
        ros_msg=Velocity,
        from_mcu=False
    )
}
