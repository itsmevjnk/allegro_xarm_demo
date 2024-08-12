import rospy
import math

from xarm_msgs.srv import *
from xarm_msgs.msg import *

from std_srvs.srv import Trigger, TriggerResponse
from arm_controller.srv import *
from arm_controller.msg import *

ARM_HOME = [-1.5707963705062866, -0.6283185482025146, 0.0, 0.0, 3.1415927410125732, 0.9599310755729675, 1.8168877363204956] # initial arm joint angles
ARM_JOINT_MAXVEL = 0.4 # rad/s
ARM_JOINT_MAXACC = 7.5 # rad/s^2
ARM_MOVE_MAXVEL = 400.0 # mm/s
ARM_MOVE_MAXACC = 4000.0 # mm/s^2

class xArmController:
    def set_blocking(self, state):
        rospy.logdebug(f'xArm: set command blocking to {state}')
        rospy.set_param('/xarm/wait_for_finish', state)

    def get_blocking(self) -> bool:
        return rospy.get_param('/xarm/wait_for_finish')

    def interrupt_move(self):
        rospy.logdebug(f'xArm: interrupting current movement')
        self.spx_set_state(4) # stop (also clears command cache)
        self.spx_set_state(0) # ready

    def move_joint(self, angles: 'list[float]'):
        rospy.logdebug(f'xArm: moving joints to {angles}')
        self.spx_move_joint(angles, ARM_JOINT_MAXVEL, ARM_JOINT_MAXACC, 0, 0)

    def home(self):
        self.move_joint(ARM_HOME)

    def move(self, pos: 'list[float]', orient: 'list[float]'):
        rospy.logdebug(f'xArm: moving to pos {pos} (mm), orientation {orient} (rad)')
        self.spx_move_base(pos + orient, ARM_MOVE_MAXVEL, ARM_MOVE_MAXACC, 0, 0)
    
    def move_tool(self, d_pos: 'list[float]', d_orient: 'list[float]'):
        rospy.logdebug(f'xArm: moving to delta pos {d_pos} (mm), orientation {d_orient} (rad) rel/TCP')
        self.spx_move_tool(d_pos + d_orient, ARM_MOVE_MAXVEL, ARM_MOVE_MAXACC, 0, 0)
    
    def set_blocking_cb(self, req):
        self.set_blocking(req.value)
        return SetBoolResponse(True, '')
    
    def get_blocking_cb(self, req):
        return GetBoolResponse(True, '', self.get_blocking())
    
    def interrupt_move_cb(self, req):
        self.interrupt_move()
        return TriggerResponse(True, '')
    
    def move_joint_cb(self, req):
        self.move_joint(req.angles)
        return MoveArmJointsResponse(True, '')
    
    def move_cb(self, req):
        self.move(req.pos, req.orient)
        return MoveArmResponse(True, '')
    
    def move_tool_cb(self, req):
        self.move_tool(req.pos, req.orient)
        return MoveArmResponse(True, '')
    
    def home_cb(self, req):
        self.home()
        return TriggerResponse(True, '')

    def telemetry_cb(self, data):
        self.last_status = ArmStatus(
            data.header,
            data.pose[0:3],
            data.pose[3:],
            data.angle,
            data.state == 1
        )
        self.status_pub.publish(self.last_status)
    
    def status_cb(self, req):
        if req.new: # wait for new message
            if self.last_status is None: # no message yet
                rospy.logdebug('xArm: waiting for first status message')
                while self.last_status is None: pass
            else: # wait for new message
                old_seq = self.last_status.header.seq
                rospy.logdebug(f'xArm: waiting for new status message (old seq = {old_seq})')
                while self.last_status.header.seq == old_seq: pass
        
        if self.last_status is None: return GetArmStatusResponse(False, 'No status messages', None)
        else: return GetArmStatusResponse(True, '', self.last_status)
            

    def __init__(self, blocking=True):
        rospy.loginfo('xArm: waiting for services to come online')
        rospy.wait_for_service('/xarm/motion_ctrl')
        rospy.wait_for_service('/xarm/set_mode')
        rospy.wait_for_service('/xarm/set_state')
        rospy.wait_for_service('/xarm/move_joint')
        rospy.wait_for_service('/xarm/move_line')
        rospy.wait_for_service('/xarm/move_line_tool')

        rospy.loginfo('xArm: setting up service proxies')
        self.spx_move_joint = rospy.ServiceProxy('/xarm/move_joint', Move)
        self.spx_move_base = rospy.ServiceProxy('/xarm/move_line', Move) # absolute
        self.spx_move_tool = rospy.ServiceProxy('/xarm/move_line_tool', Move) # relative
        self.spx_set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)

        rospy.loginfo('xArm: initialising')
        rospy.ServiceProxy('/xarm/motion_ctrl', SetAxis)(8, 1) # unlock all servos
        rospy.ServiceProxy('/xarm/set_mode', SetInt16)(0) # robot mode 0
        self.spx_set_state(0) # get ready

        self.set_blocking(True) # synchronous operation during init

        rospy.loginfo('xArm: moving to home position')
        self.home()

        rospy.loginfo(f'xArm: movement service blocking set to {blocking}')
        self.set_blocking(blocking)

        rospy.loginfo('xArm: setting up services')
        self.srv_set_blocking = rospy.Service('/arm/set_blocking', SetBool, self.set_blocking_cb)
        self.srv_get_blocking = rospy.Service('/arm/get_blocking', GetBool, self.get_blocking_cb)
        self.srv_interrupt_move = rospy.Service('/arm/intr_move', Trigger, self.interrupt_move_cb)
        self.srv_move_joint = rospy.Service('/arm/move_joint', MoveArmJoints, self.move_joint_cb)
        self.srv_move = rospy.Service('/arm/move', MoveArm, self.move_cb)
        self.srv_move_tool = rospy.Service('/arm/move_tool', MoveArm, self.move_tool_cb)
        self.srv_home = rospy.Service('/arm/home', Trigger, self.home_cb)
        self.srv_status = rospy.Service('/arm/status', GetArmStatus, self.status_cb)

        rospy.loginfo('xArm: setting up status topic')
        self.status_pub = rospy.Publisher('/arm/status', ArmStatus, queue_size = 10)
        # self.jointpos_pub = rospy.Publisher('/arm/joint_pos', JointVal, queue_size = 10)
        
        rospy.loginfo('xArm: subscribing to telemetry')
        self.last_status = None # OUR last status message
        self.telemetry_sub = rospy.Subscriber('/xarm/xarm_states', RobotMsg, self.telemetry_cb)

if __name__ == '__main__':
    rospy.init_node('xarm_controller')

    xArmController()
    rospy.spin()