import roslib
roslib.load_manifest("pr2_simple_arm_motions")
from pr2_simple_arm_motions import GripUtils
import StanceUtils
from smach import State, StateMachine
from smach_ros import SimpleActionState
from numpy import pi,abs
from geometry_msgs.msg import PointStamped
import rospy
SUCCESS = 'success'
FAILURE = 'failure'

DEFAULT_OUTCOMES = [SUCCESS, FAILURE]
MINIMUM_SHAKE_WIDTH = 0.4

class StateMachineAddition(object):
    def __init__(self, title, state, transitions=None, remapping=None):
        self.title = title
        self.state = state
        self.transitions = transitions
        self.remapping = remapping 

class NestedStateMachine(object):
    def __init__(self, title=None, transitions=None, state_machine_additions=[], outcomes=DEFAULT_OUTCOMES, state_machines=[], things_to_add = [], input_keys = [], output_keys = [], remapping = None):
        self.title = title
        self.transitions = transitions
        self.state_machine_additions = list(state_machine_additions)
        self.outcomes = outcomes
        self.state_machines = list(state_machines)
        self.things_to_add = list(things_to_add)
        self.input_keys = input_keys
        self.output_keys = output_keys
        self.remapping = remapping
        
    def add(self,title,state,transitions=None,remapping=None):
        if isinstance(state,NestedStateMachine):
            state.title = title
            state.transitions = transitions
            state.remapping = remapping
            return self.add_state_machine(state)
        else:
            return self.add_state(title,state,transitions,remapping)
    
    def add_state(self,title, state, transitions=None, remapping=None):
        self.state_machine_additions.append(StateMachineAddition(title, state, transitions, remapping))
        self.things_to_add.append('s')
    def add_state_machine(self, state_machine):
        self.state_machines.append(state_machine)
        self.things_to_add.append('m')
    def add_states(self):
        sm_nested = StateMachine(outcomes=self.outcomes, input_keys=self.input_keys, output_keys=self.output_keys)
        machine_count = 0
        state_count = 0
        StateMachine.add(self.title,sm_nested,transitions=self.transitions, remapping=self.remapping)
        with sm_nested:
            for thing in self.things_to_add:
                if thing == 's':
                    addition = self.state_machine_additions[state_count]
                    StateMachine.add(addition.title, addition.state, addition.transitions, remapping=addition.remapping)
                    state_count += 1
                else:
                    machine = self.state_machines[machine_count]
                    machine.add_states()
                    machine_count += 1
                    

class OuterStateMachine(StateMachine):
    @staticmethod
    def add(title,state,transitions=None,remapping=None):
        if isinstance(state,NestedStateMachine):
            return OuterStateMachine.add_state_machine(title,state,transitions,remapping)
        else:
            return StateMachine.add(title,state,transitions,remapping)
    @staticmethod
    def add_state_machine(title, state_machine, transitions=None,remapping=None):
        state_machine.title = title
        if transitions:
            state_machine.transitions = transitions
        state_machine.add_states()

class ArmsUpThenShakeMachine(NestedStateMachine):
    def __init__(self, title=None, transitions=None):
        NestedStateMachine.__init__(self, title, transitions=transitions)
        self.add_state('ArmsUp', ArmsUp(), {SUCCESS:'Shake', FAILURE:FAILURE})
        self.add_state('Shake', ShakeOneArm('l',2), {SUCCESS:SUCCESS, FAILURE:FAILURE})

class SuccessFailureState(State):
    def __init__(self,input_keys=[],output_keys=[]):
        State.__init__(self, DEFAULT_OUTCOMES,input_keys=input_keys,output_keys=output_keys)


class Smooth(SuccessFailureState):
    def __init__(self, arm="b", input_keys=['location', 'distance']):
        SuccessFailureState.__init__(self, input_keys=input_keys)
        self.arm = arm
    def execute(self, userdata):
        self.location = userdata.location
        self.distance = userdata.distance
        initial_separation = 0.11
        if self.arm=="b":
            #Put arms together, with a gap of initial_separation between grippers
            if not GripUtils.go_to_pts(point_l=self.location,grip_r=True, grip_l=True, point_r=self.location,
                    roll_l=pi/2,yaw_l=0,pitch_l=-pi/2,y_offset_l=initial_separation/2.0,z_offset_l=0.05
                    ,link_frame_l="l_wrist_back_frame",
                    roll_r=pi/2,yaw_r=0,pitch_r=-pi/2,y_offset_r=-1*initial_separation/2.0,z_offset_r=0.05
                    ,link_frame_r="r_wrist_back_frame",dur=4.0):
                return FAILURE
            if not GripUtils.go_to_pts(point_l=self.location,grip_r=True, grip_l=True, point_r=self.location,
                    roll_l=pi/2,yaw_l=0,pitch_l=-pi/2,y_offset_l=initial_separation/2.0,z_offset_l=-0.03, 
                    link_frame_l="l_wrist_back_frame",
                    roll_r=pi/2,yaw_r=0,pitch_r=-pi/2,y_offset_r=-1*initial_separation/2.0,z_offset_r=-0.03, 
                    link_frame_r="r_wrist_back_frame",dur=2.0):
                return FAILURE
            if not GripUtils.go_to_pts(point_l=self.location,grip_r=True, grip_l=True, point_r=self.location,
                    roll_l=pi/2,yaw_l=0,pitch_l=-pi/2,
                    y_offset_l=(self.distance+initial_separation)/2.0, z_offset_l=-0.03,
                    link_frame_l="l_wrist_back_frame",
                    roll_r=pi/2,yaw_r=0,pitch_r=-pi/2,
                    y_offset_r=-1*(self.distance+initial_separation)/2.0, z_offset_r=-0.03,
                    link_frame_r="r_wrist_back_frame",dur=2.0):
                return FAILURE
        else:
            #Right is negative in the y axis
            if self.arm=="r":
                y_multiplier = -1
            else:
                y_multiplier = 1.0
            if not GripUtils.go_to_pt(point=self.location,grip=True,roll=pi/2,yaw=0,pitch=-pi/2,
                    z_offset=0.05,arm=self.arm,
                    link_frame="%s_wrist_back_frame"%self.arm,dur=4.0):
                return FAILURE
            if not GripUtils.go_to_pt(point=self.location,grip=True,roll=pi/2,yaw=0,pitch=-pi/2,
                    z_offset=-0.01,arm=self.arm,
                    link_frame="%s_wrist_back_frame"%self.arm,dur=2.0):
                return FAILURE
            if not GripUtils.go_to_pt(point=self.location,grip=True,roll=pi/2,yaw=0,pitch=-pi/2,
                    y_offset=y_multiplier*self.distance,z_offset=-0.00,arm=self.arm,
                    link_frame="%s_wrist_back_frame"%self.arm,dur=2.0):
                return FAILURE
        return SUCCESS
        
class SmoothOnTable(SuccessFailureState):
    def __init__(self, arm="b", smooth_x=0.5, smooth_y=0.0,distance=None):
        SuccessFailureState.__init__(self)
        self.arm = arm
        self.location = PointStamped()
        self.location.point.x = smooth_x
        self.location.point.y = smooth_y
        self.location.point.z = 0.0
        self.location.header.frame_id = "table_height"
        if not distance:
            self.distance = abs(smooth_y)
        else:
            self.distance = distance
    def execute(self, userdata):

        initial_separation = 0.11
        print 'Smooth distance: %d' % self.distance
        if self.arm=="b":
            rospy.loginfo('Self.arm is b')
            outcome = SUCCESS
            #Put arms together, with a gap of initial_separation between grippers
            if not GripUtils.go_to_pts(point_l=self.location,grip_r=True, grip_l=True, point_r=self.location,
                    roll_l=pi/2,yaw_l=0,pitch_l=-pi/2,y_offset_l=initial_separation/2.0,z_offset_l=0.05
                    ,link_frame_l="l_wrist_back_frame",
                    roll_r=pi/2,yaw_r=0,pitch_r=-pi/2,y_offset_r=-1*initial_separation/2.0,z_offset_r=0.05
                    ,link_frame_r="r_wrist_back_frame",dur=4.0):
                outcome = FAILURE
            rospy.loginfo('Smooth on table goto 1: Success is %s', outcome==SUCCESS)
            if not GripUtils.go_to_pts(point_l=self.location,grip_r=True, grip_l=True, point_r=self.location,
                    roll_l=pi/2,yaw_l=0,pitch_l=-pi/2,y_offset_l=initial_separation/2.0,z_offset_l=0.00,
                    link_frame_l="l_wrist_back_frame",
                    roll_r=pi/2,yaw_r=0,pitch_r=-pi/2,y_offset_r=-1*initial_separation/2.0,z_offset_r=0.00,
                    link_frame_r="r_wrist_back_frame",dur=2.0):
                outcome = FAILURE
            rospy.loginfo('Smooth on table goto 2: Success is %s', outcome==SUCCESS)
            if not GripUtils.go_to_pts(point_l=self.location,grip_r=True, grip_l=True, point_r=self.location,
                    roll_l=pi/2,yaw_l=0,pitch_l=-pi/2,
                    y_offset_l=(self.distance+initial_separation)/2.0, z_offset_l=0.00,
                    link_frame_l="l_wrist_back_frame",
                    roll_r=pi/2,yaw_r=0,pitch_r=-pi/2,
                    y_offset_r=-1*(self.distance+initial_separation)/2.0, z_offset_r=0.00,
                    link_frame_r="r_wrist_back_frame",dur=2.0):
                outcome = FAILURE
            rospy.loginfo('Smooth on table goto 3: Success is %s', outcome==SUCCESS)
            GripUtils.recall_arm("b")
            return outcome
        else:
            #Right is negative in the y axis
            if self.arm=="r":
                y_multiplier = -1
            else:
                y_multiplier = 1
            rospy.loginfo('arm is %s, y multiplier is %s' % (self.arm, y_multiplier))
            print 'Location: %s' % str(self.location)
            if not GripUtils.go_to_pt(point=self.location,grip=True,roll=pi/2,yaw=0,pitch=-pi/2,
                    z_offset=0.05,arm=self.arm,
                    link_frame="%s_wrist_back_frame"%self.arm,dur=4.0, verbose=True):
                return FAILURE
            rospy.loginfo('Step 2')
            if not GripUtils.go_to_pt(point=self.location,grip=True,roll=pi/2,yaw=0,pitch=-pi/2,
                    z_offset=-0.01,arm=self.arm,
                    link_frame="%s_wrist_back_frame"%self.arm,dur=2.0, verbose=True):
                #return FAILURE
                pass
            rospy.loginfo('Step 3')
            print 'Offset: %f' % (y_multiplier*self.distance) 
            if not GripUtils.go_to_pt(point=self.location,grip=True,roll=pi/2,yaw=0,pitch=-pi/2,
                    y_offset=y_multiplier*self.distance,z_offset=-0.01,arm=self.arm,
                    link_frame="%s_wrist_back_frame"%self.arm,dur=2.0, verbose=True):
                return FAILURE
            rospy.loginfo('Step 4.  Done!')
        return SUCCESS    

class SpreadOut(SuccessFailureState):
    
    def __init__(self,forward_amount,recall=False):
        SuccessFailureState.__init__(self,input_keys=['cloth_width','cloth_height'])
        self.forward_amount = forward_amount
        self.recall = recall
        
    def execute(self,userdata):
        #cloth_width = userdata.cloth_width*.85
        cloth_width = userdata.cloth_width * 0.925
        cloth_height = userdata.cloth_height
        start_x = 0.20
        end_x  = start_x + self.forward_amount
        down_height = 0.03
        up_height          = 0.625
        return_val = SUCCESS
        if not GripUtils.go_to_multi(x_l=start_x,y_l=cloth_width/2.0,z_l=up_height,
                                        roll_l=0,pitch_l=0,yaw_l=-pi/2,grip_l=True,
                                        x_r=start_x,y_r=-cloth_width/2.0,z_r=up_height,
                                        roll_r=0,pitch_r=0,yaw_r=pi/2,grip_r=True,
                                        frame_l="table_height",frame_r="table_height",dur=4.0):
            return_val = FAILURE
        if not cloth_height:
        
            if not GripUtils.go_to_multi(x_l=start_x,y_l=cloth_width/2.0,z_l=(0.75*up_height + 0.25*down_height),
                                            roll_l=0,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,
                                            x_r=start_x,y_r=-cloth_width/2.0,z_r=(0.75*up_height + 0.25*down_height),
                                            roll_r=0,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,
                                            frame_l="table_height",frame_r="table_height",dur=3.0):
                return_val = FAILURE
            if not GripUtils.go_to_multi(x_l=(start_x + end_x)/2.0,y_l=cloth_width/2.0,z_l=0.25*up_height + 0.75*down_height,
                                            roll_l=0,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,
                                            x_r=(start_x+end_x)/2.0,y_r=-cloth_width/2.0,z_r=0.25*up_height + 0.75*down_height,
                                            roll_r=0,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,
                                            frame_l="table_height",frame_r="table_height",dur=3.0):
                return_val = FAILURE
            
            if not GripUtils.go_to_multi(x_l=end_x-0.1,y_l=cloth_width/2.0,z_l=0.25*up_height + 0.75*down_height,
                                        roll_l=0,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,
                                        x_r=end_x-0.1,y_r=-cloth_width/2.0,z_r=0.25*up_height + 0.75*down_height,
                                        roll_r=0,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,
                                        frame_l="table_height",frame_r="table_height",dur=3.0):
                return_val = FAILURE
            
            if not GripUtils.go_to_multi(x_l=end_x+0.1,y_l=cloth_width/2.0,z_l=down_height+0.03,
                                        roll_l=0,pitch_l=pi/4,yaw_l=-pi/5,grip_l=True,
                                        x_r=end_x+0.1,y_r=-cloth_width/2.0,z_r=down_height+0.03,
                                        roll_r=0,pitch_r=pi/4,yaw_r=pi/5,grip_r=True,
                                        frame_l="table_height",frame_r="table_height",dur=3.0):
                return_val = FAILURE
            
            
            
        else:
            if not GripUtils.go_to_multi(x_l=start_x,y_l=cloth_width/2.0,z_l=cloth_height+down_height,
                                            roll_l=0,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,
                                            x_r=start_x,y_r=-cloth_width/2.0,z_r=cloth_height+down_height,
                                            roll_r=0,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,
                                            frame_l="table_height",frame_r="table_height",dur=3.0):
                return_val = FAILURE
            if not GripUtils.go_to_multi(x_l=end_x - 0.1 - cloth_height/2.0,y_l=cloth_width/2.0,z_l=cloth_height/2.0+down_height+0.03,
                                            roll_l=0,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,
                                            x_r=end_x - 0.1 - cloth_height/2.0,y_r=-cloth_width/2.0,z_r=cloth_height/2.0+down_height+0.03,
                                            roll_r=0,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,
                                            frame_l="table_height",frame_r="table_height",dur=3.0):
                return_val = FAILURE
            
            
            
            if not GripUtils.go_to_multi(x_l=end_x-0.1,y_l=cloth_width/2.0,z_l=down_height+0.03,
                                        roll_l=0,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,
                                        x_r=end_x-0.1,y_r=-cloth_width/2.0,z_r=down_height+0.03,
                                        roll_r=0,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,
                                        frame_l="table_height",frame_r="table_height",dur=3.0):
                return_val = FAILURE
            cloth_width *= 1.05
            if not GripUtils.go_to_multi(x_l=end_x+0.1,y_l=cloth_width/2.0,z_l=down_height+0.03,
                                        roll_l=0,pitch_l=pi/4,yaw_l=-pi/5,grip_l=True,
                                        x_r=end_x+0.1,y_r=-cloth_width/2.0,z_r=down_height+0.03,
                                        roll_r=0,pitch_r=pi/4,yaw_r=pi/5,grip_r=True,
                                        frame_l="table_height",frame_r="table_height",dur=3.0):
                return_val = FAILURE
        
        
        if self.recall:
            GripUtils.open_gripper("b")
            GripUtils.recall_arm("b")
        return SUCCESS #was return_val
        
    

class SpreadAcross(NestedStateMachine):
    def __init__(self, direction, table_width, lift_amount,title=None, transitions=None):
        NestedStateMachine.__init__(self, title, transitions=transitions)
        forward_amount = 0.4
        self.add_state('Sweep',Sweep(direction, drag_amount=table_width*0.95,table_width=table_width,lift_amount=lift_amount,forward_amount=forward_amount),{SUCCESS:'Smooth',FAILURE:FAILURE})
        smooth_x = forward_amount+0.05
        smooth_y = 0.1
        distance = table_width * 0.5
        if direction == 'r':
            smootharm = 'l'
            smooth_y *=-1
        else:
            smootharm = 'r'

        self.add_state('Smooth',SmoothOnTable(smootharm,smooth_x = smooth_x, smooth_y = smooth_y, distance=distance),{SUCCESS:'Recall',FAILURE:'Recall'})
        self.add_state('Recall',RecallArm(smootharm),{SUCCESS:SUCCESS,FAILURE:FAILURE})

class Sweep(SuccessFailureState):
    def __init__(self,direction,drag_amount,table_width,lift_amount,forward_amount=0.5):
        SuccessFailureState.__init__(self)
        self.direction = direction
        self.drag_amount = drag_amount
        self.table_width = table_width
        self.lift_amount = lift_amount
        self.forward_amount=forward_amount
        #Testing
        self.table_width += 0.04
    def execute(self, userdata):
        
        forward_amount = self.forward_amount
        hover_amount = 0.06
        if self.direction == "r":
            drag_arm = "r"
            edge_y = self.table_width / 2.0
            drag_yaw = pi/2
            final_y = edge_y - self.drag_amount
        else:
            drag_arm = "l"
            edge_y = -1 * self.table_width / 2.0
            drag_yaw = -pi/2
            final_y = edge_y + self.drag_amount

        #roll = 0
        roll = pi/2

        GripUtils.go_to(x=forward_amount,y=edge_y,z=self.lift_amount,
                roll=roll,pitch=0,yaw=drag_yaw,grip=True,
                frame="table_height",arm=drag_arm,dur=2.0)
        rospy.sleep(1.0) #Give time to stabilize
        if not GripUtils.go_to(x=forward_amount,y=edge_y,z=hover_amount,
                roll=roll,pitch=pi/4,yaw=drag_yaw,grip=True,
                frame="table_height",arm=drag_arm,dur=2.0):
            return FAILURE
        
        
        if not GripUtils.go_to(x=forward_amount,y=(edge_y+final_y)/2.0,z=hover_amount,
                roll=roll,pitch=pi/4,yaw=drag_yaw,grip=True,
                frame="table_height",arm=drag_arm,dur=2.5):
            return FAILURE
        
        if not GripUtils.go_to(x=forward_amount,y=final_y,z=hover_amount,
                roll=roll,pitch=pi/4,yaw=drag_yaw,grip=True,
                frame="table_height",arm=drag_arm,dur=2.5):
            return FAILURE
        return SUCCESS

class SweepSameSide(SuccessFailureState):
    def __init__(self,direction,drag_amount,table_width,lift_amount):
        SuccessFailureState.__init__(self)
        self.direction = direction
        self.drag_amount = drag_amount
        self.table_width = table_width
        self.lift_amount = lift_amount
    def execute(self, userdata):
        forward_amount = 0.5
        hover_amount = 0.06
        if self.direction == "r":
            drag_arm = "r"
            edge_y = -1.0 * self.table_width / 2.0
            drag_yaw = pi/2
            final_y = edge_y + self.drag_amount
        else:
            drag_arm = "l"
            edge_y = 1.0 * self.table_width / 2.0
            drag_yaw = -pi/2
            final_y = edge_y - self.drag_amount

        if not GripUtils.go_to(x=forward_amount*.8,y=edge_y*1.25,z=self.lift_amount,
                roll=0,pitch=0,yaw=drag_yaw/4,grip=True,
                frame="table_heig@staticmethodht",arm=drag_arm):
            print 'f1'
            return FAILURE
        if not GripUtils.go_to(x=forward_amount,y=edge_y,z=hover_amount,
                roll=0,pitch=pi/4,yaw=drag_yaw,grip=True,
                frame="table_height",arm=drag_arm):
            print 'f2'
            return FAILURE

        if not GripUtils.go_to(x=forward_amount,y=final_y,z=hover_amount,
                roll=0,pitch=pi/2,yaw=drag_yaw,grip=True,
                frame="table_height",arm=drag_arm):
            print 'f3'
            return FAILURE
        return SUCCESS

class ShakeOneArm(SuccessFailureState):
    def __init__(self,arm,num_shakes):
        SuccessFailureState.__init__(self)
        self.arm = arm 
        if num_shakes < 1:
            rospy.logwarn("Must shake at least once.")
            num_shakes = 1
        self.num_shakes  = num_shakes

    def execute(self, userdata):

        forward_amount  = 0.375
        height          = 0.65
        drop_amount     = 0.25
        lateral_amount  = 0.05

        if self.arm=="r":
            yaw_multiplier = 1
            y_multiplier = -1
        else:
            yaw_multiplier = -1
            y_multiplier = 1
        
        for i in range(self.num_shakes):
            if i == 0:
                duration = 4.0
            if not GripUtils.go_to(x=forward_amount,y=0,z=height,
                    roll=0,pitch=0,yaw=yaw_multiplier*pi/2,grip=True,
                    frame="table_height",arm=self.arm,dur=duration):
                return FAILURE
            duration = .5
            if not GripUtils.go_to(x=forward_amount,y=lateral_amount*y_multiplier,z=height-drop_amount,
                    roll=0,pitch=pi/4,yaw=yaw_multiplier*pi/2,grip=True,
                    frame="table_height",arm=self.arm,dur=duration):
                return FAILURE
        if not GripUtils.go_to(x=forward_amount,y=0,z=height,
            roll=0,pitch=0,yaw=yaw_multiplier*pi/2,grip=True,
                frame="table_height",arm=self.arm,dur=duration):
            return FAILURE
        return SUCCESS

class ShakeBothArms(SuccessFailureState):
    def __init__(self,num_shakes, violent = True, input_keys=['cloth_width']):
        SuccessFailureState.__init__(self, input_keys=input_keys)
        if num_shakes < 1:
            rospy.logwarn("Must shake at least once.")
            num_shakes = 1
        self.num_shakes  = num_shakes
        self.violent = violent

    def execute(self, userdata):

        cloth_width     = userdata.cloth_width*.85

        if cloth_width < MINIMUM_SHAKE_WIDTH:
            rospy.logwarn("Cannot shake at less than %s m apart; tried to do %d" % (MINIMUM_SHAKE_WIDTH, cloth_width))
            return FAILURE

        forward_amount  = 0.45
        height          = 0.625
        if self.violent:
            drop_amount = 0.3
        else:
            drop_amount = 0.1
        lateral_amount  = 0.1

        for i in range(self.num_shakes):
            if i == 0:
                duration = 4.0
            if not GripUtils.go_to_multi(x_l=forward_amount,y_l=cloth_width/2.0,z_l=height,
                                        roll_l=0,pitch_l=0,yaw_l=-pi/2,grip_l=True,
                                        x_r=forward_amount,y_r=-cloth_width/2.0,z_r=height,
                                        roll_r=0,pitch_r=0,yaw_r=pi/2,grip_r=True,
                                        frame_l="table_height",frame_r="table_height",dur=duration):
                return_val = FAILURE

            duration = .6
            return_val = SUCCESS
            if not GripUtils.go_to_multi(x_l=forward_amount,y_l=cloth_width/2.0-lateral_amount,z_l=height-drop_amount,
                                        roll_l=0,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,
                                        x_r=forward_amount,y_r=-cloth_width/2.0+lateral_amount,z_r=height-drop_amount,
                                        roll_r=0,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,
                                        frame_l="table_height",frame_r="table_height",dur=duration):
                return_val = FAILURE
        if not GripUtils.go_to_multi(x_l=forward_amount,y_l=cloth_width/2.0,z_l=height,
                                        roll_l=0,pitch_l=0,yaw_l=-pi/2,grip_l=True,
                                        x_r=forward_amount,y_r=-cloth_width/2.0,z_r=height,
                                        roll_r=0,pitch_r=0,yaw_r=pi/2,grip_r=True,
                                        frame_l="table_height",frame_r="table_height",dur=duration):
            return_val = FAILURE

        return SUCCESS

class ArmsUp(SuccessFailureState):
    def __init__(self,grip=True):
        SuccessFailureState.__init__(self)
        self.grip = grip
    def execute(self, userdata):
        height = 0.35
        lateral_amount = 0.65
        forward_amount = 0.3

        if not GripUtils.go_to_multi(   x_l=forward_amount, y_l=lateral_amount, z_l=height,
                                        roll_l=0, pitch_l=0, yaw_l=0, grip_l=self.grip,
                                        x_r=forward_amount, y_r=-lateral_amount, z_r=height,
                                        roll_r=0, pitch_r=0, yaw_r=0, grip_r=self.grip,
                                        frame_l="torso_lift_link", frame_r="torso_lift_link", dur=4.0):
            return FAILURE
        else:
            return SUCCESS

class RecallArm(SuccessFailureState):
    def __init__(self, arm):
        SuccessFailureState.__init__(self)
        self.arm = arm
    def execute(self, userdata):

        height = 0.35
        lateral_amount = 0.65
        forward_amount = 0.3

        if self.arm == 'b':
            lateral_amount_r = lateral_amount * -1
            lateral_amount_l = lateral_amount

            if not GripUtils.go_to_multi(   x_l=forward_amount,         y_l=lateral_amount_l,   z_l=height,
                                            roll_l=0,                   pitch_l=0,              yaw_l=0, 
                                            frame_l="torso_lift_link",  grip_l=False,
                                            x_r=forward_amount,         y_r=lateral_amount_r,   z_r=height,
                                            roll_r=0,                   pitch_r=0,              yaw_r=0,
                                            frame_r="torso_lift_link",  grip_r=False,           dur=4.0):
                return FAILURE
            else:
                return SUCCESS
        else:
            if self.arm == 'r':
                lateral_amount = lateral_amount * -1
            if not GripUtils.go_to(     x=forward_amount, y=lateral_amount, z=height,
                                        roll=0, pitch=0, yaw=0, grip=False,
                                        frame="torso_lift_link", dur=4.0, arm=self.arm):
                return FAILURE
            else:
                return SUCCESS
                



class OpenGrippers(SuccessFailureState):
    def __init__(self, arm='b'):
        SuccessFailureState.__init__(self)
        self.arm = arm
    def execute(self, userdata):
        if not GripUtils.open_gripper(self.arm):
            return FAILURE
        else:
            return SUCCESS

class CloseGrippers(SuccessFailureState):
    def __init__(self, arm='b'):
        self.arm = arm
    def execute(self, userdata):
        if not GripUtils.close_gripper(self.arm):
            return FAILURE
        else:
            return SUCCESS
            
class StanceState(SuccessFailureState):
    def __init__(self,stance_name, dur=5.0):
        SuccessFailureState.__init__(self)
        self.stance_name = stance_name
        self.dur = dur
        
    def execute(self,userdata):
        if not StanceUtils.call_stance(self.stance_name,self.dur):
            return FAILURE
        else:
            return SUCCESS
