#!/usr/bin/python
import rospy
import smach
import sys
import time

from cb_planner_msgs_srvs.msg import PositionConstraint

from robot_smach_states.util.designators import VariableDesignator, EdEntityDesignator, EntityByIdDesignator, analyse_designators
import robot_smach_states as states

from robocup_knowledge import load_knowledge

challenge_knowledge = load_knowledge('demo_ED')

print "=============================================="
print "==                 DEMO ED                  =="
print "=============================================="


class checkTimeOut(smach.State):
    def __init__(self, robot, time_out_seconds):
        smach.State.__init__(self, outcomes=["not_yet", "time_out"])
        self.robot = robot
        self.time_out_seconds = time_out_seconds

        self.start = None
        self.last_say = None

        self.turn = -1

    def execute(self, userdata):
        current_seconds = rospy.Time.now().to_sec()

        radians = 0.15
        vth = 0.5
        if self.start is None:
            self.robot.base.force_drive(0, 0, vth, radians / vth)
            self.start = current_seconds

        dt = current_seconds - self.start

        if dt > self.time_out_seconds:
            return "time_out"

        if self.last_say is None or current_seconds - self.last_say > 10:
            self.robot.speech.speak("Trying for another %d seconds .. wiggle wiggle" % int(self.time_out_seconds - dt),
                                    block=False)
            self.last_say = current_seconds

        self.robot.base.force_drive(0, 0, self.turn * vth, (2 * radians) / vth)
        self.turn = -self.turn

        return "not_yet"


class Turn(smach.State):
    def __init__(self, robot, radians):
        smach.State.__init__(self, outcomes=["turned"])
        self.robot = robot
        self.radians = radians

    def execute(self, userdata):
        self.robot.head.close()

        vth = 1.0
        print "Turning %f radians with force drive" % self.radians
        self.robot.base.force_drive(0, 0, vth, self.radians / vth)

        return "turned"


def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:
        # Start challenge via StartChallengeRobust
        smach.StateMachine.add("START_DEMO",
                               states.Initialize(robot),
                               transitions={"initialized": "SAY_GOTO_TARGET1",
                                            "abort": "SAY_GOTO_TARGET1"})

        smach.StateMachine.add('SAY_GOTO_TARGET1',
                               states.Say(robot, ["I will go to target 1 now",
                                                  "I will now go to target 1",
                                                  "Lets go to target 1",
                                                  "Going to target 1"], block=False),
                               transitions={'spoken': 'GOTO_TARGET1'})

        ######################################################################################################################################################
        #
        #                                                       TARGET 1
        #
        ######################################################################################################################################################

        smach.StateMachine.add('GOTO_TARGET1',
                               states.NavigateToWaypoint(robot,
                                                         EntityByIdDesignator(robot, id=challenge_knowledge.target1),
                                                         challenge_knowledge.target1_radius1, look_at_designator=False),
                               transitions={'arrived': 'SAY_TARGET1_REACHED',
                                            'unreachable': 'RESET_ED_TARGET1',
                                            'goal_not_defined': 'RESET_ED_TARGET1'})

        smach.StateMachine.add('SAY_TARGET1_REACHED',
                               states.Say(robot, ["Reached target 1",
                                                  "I have arrived at target 1",
                                                  "I am now at target 1"], block=True),
                               transitions={'spoken': 'SAY_GOTO_TARGET2'})

        smach.StateMachine.add('RESET_ED_TARGET1',
                               states.ResetED(robot),
                               transitions={'done': 'GOTO_TARGET1_BACKUP'})

        smach.StateMachine.add('GOTO_TARGET1_BACKUP',
                               states.NavigateToWaypoint(robot,
                                                         EntityByIdDesignator(robot, id=challenge_knowledge.target1),
                                                         challenge_knowledge.target1_radius2, look_at_designator=False),
                               transitions={'arrived': 'SAY_TARGET1_REACHED',
                                            'unreachable': 'TIMEOUT1',
                                            'goal_not_defined': 'TIMEOUT1'})

        smach.StateMachine.add('TIMEOUT1',
                               checkTimeOut(robot, challenge_knowledge.time_out_seconds),
                               transitions={'not_yet': 'GOTO_TARGET1', 'time_out': 'SAY_TARGET1_FAILED'})

        # Should we mention that we failed???
        smach.StateMachine.add('SAY_TARGET1_FAILED',
                               states.Say(robot, ["I am not able to reach target 1",
                                                  "I cannot reach target 1",
                                                  "Target 1 is unreachable"], block=True),
                               transitions={'spoken': 'SAY_GOTO_TARGET2'})

        smach.StateMachine.add('SAY_GOTO_TARGET2',
                               states.Say(robot, ["I will go to target 2 now",
                                                  "I will now go to target 2",
                                                  "Lets go to target 2",
                                                  "Going to target 2"], block=False),
                               transitions={'spoken': 'GOTO_TARGET2'})

        ######################################################################################################################################################
        #
        #                                                       TARGET 2
        #
        ######################################################################################################################################################

        smach.StateMachine.add('GOTO_TARGET2',
                               states.NavigateToWaypoint(robot,
                                                         EntityByIdDesignator(robot, id=challenge_knowledge.target2),
                                                         challenge_knowledge.target2_radius1, look_at_designator=False),
                               transitions={'arrived': 'SAY_TARGET2_REACHED',
                                            'unreachable': 'RESET_ED_TARGET2',
                                            'goal_not_defined': 'RESET_ED_TARGET2'})

        smach.StateMachine.add('SAY_TARGET2_REACHED',
                               states.Say(robot, ["Reached target 2",
                                                  "I have arrived at target 2",
                                                  "I am now at target 2"], block=True),
                               transitions={'spoken': 'SAY_GOTO_TARGET3'})

        smach.StateMachine.add('RESET_ED_TARGET2',
                               states.ResetED(robot),
                               transitions={'done': 'GOTO_TARGET2_BACKUP'})

        smach.StateMachine.add('GOTO_TARGET2_BACKUP',
                               states.NavigateToWaypoint(robot,
                                                         EntityByIdDesignator(robot, id=challenge_knowledge.target2),
                                                         challenge_knowledge.target2_radius2, look_at_designator=False),
                               transitions={'arrived': 'SAY_TARGET2_REACHED',
                                            'unreachable': 'TIMEOUT2',
                                            'goal_not_defined': 'TIMEOUT2'})

        smach.StateMachine.add('TIMEOUT2',
                               checkTimeOut(robot, challenge_knowledge.time_out_seconds),
                               transitions={'not_yet': 'GOTO_TARGET2', 'time_out': 'SAY_TARGET2_FAILED'})

        # Should we mention that we failed???
        smach.StateMachine.add('SAY_TARGET2_FAILED',
                               states.Say(robot, ["I am not able to reach target 2",
                                                  "I cannot reach target 2",
                                                  "Target 2 is unreachable"], block=True),
                               transitions={'spoken': 'SAY_GOTO_TARGET3'})

        smach.StateMachine.add('SAY_GOTO_TARGET3',
                               states.Say(robot, ["I will go to target 3 now",
                                                  "I will now go to target 3",
                                                  "Lets go to target 3",
                                                  "Going to target 3"], block=False),
                               transitions={'spoken': 'GOTO_TARGET3'})

        ######################################################################################################################################################
        #
        #                                                       TARGET 3
        #
        ######################################################################################################################################################

        smach.StateMachine.add('GOTO_TARGET3',
                                states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.target3), challenge_knowledge.target3_radius1, look_at_designator=False),
                                transitions={   'arrived'           :   'SAY_TARGET3_REACHED',
                                                'unreachable'       :   'RESET_ED_TARGET3',
                                                'goal_not_defined'  :   'RESET_ED_TARGET3'})

        smach.StateMachine.add( 'SAY_TARGET3_REACHED',
                                states.Say(robot, ["Reached target 3",
                                                    "I have arrived at target 3",
                                                    "I am now at target 3"], block=True),
                                transitions={   'spoken'            :   'SAY_GOTO_TARGET4'})

        smach.StateMachine.add('RESET_ED_TARGET3',
                                states.ResetED(robot),
                                transitions={   'done'              :   'GOTO_TARGET3_BACKUP'})

        smach.StateMachine.add('GOTO_TARGET3_BACKUP',
                                states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.target3), challenge_knowledge.target3_radius2, look_at_designator=False),
                                transitions={   'arrived'           :   'SAY_TARGET3_REACHED',
                                                'unreachable'       :   'TIMEOUT3',
                                                'goal_not_defined'  :   'TIMEOUT3'})

        smach.StateMachine.add( 'TIMEOUT3',
                                checkTimeOut(robot, challenge_knowledge.time_out_seconds),
                                transitions={'not_yet': 'GOTO_TARGET3', 'time_out': 'SAY_TARGET3_FAILED'})

        # Should we mention that we failed???
        smach.StateMachine.add( 'SAY_TARGET3_FAILED',
                                states.Say(robot, ["I am not able to reach target 3",
                                                    "I cannot reach target 3",
                                                    "Target 3 is unreachable"], block=True),
                                transitions={   'spoken'            :   'SAY_GOTO_TARGET4'})

        smach.StateMachine.add('SAY_GOTO_TARGET4',
                               states.Say(robot, ["I will go to target 4 now",
                                                  "I will now go to target 4",
                                                  "Lets go to target 4",
                                                  "Going to target 4"], block=False),
                               transitions={'spoken': 'GOTO_TARGET4'})

        ######################################################################################################################################################
        #
        #                                                       TARGET 4
        #
        ######################################################################################################################################################

        smach.StateMachine.add('GOTO_TARGET4',
                               states.NavigateToWaypoint(robot,
                                                         EntityByIdDesignator(robot, id=challenge_knowledge.target4),
                                                         challenge_knowledge.target4_radius1, look_at_designator=False),
                               transitions={'arrived': 'SAY_TARGET4_REACHED',
                                            'unreachable': 'RESET_ED_TARGET4',
                                            'goal_not_defined': 'RESET_ED_TARGET4'})

        smach.StateMachine.add('SAY_TARGET4_REACHED',
                               states.Say(robot, ["Reached target 4",
                                                  "I have arrived at target 4",
                                                  "I am now at target 4"], block=True),
                               transitions={'spoken': 'SAY_GOTO_STARTING_POINT'})

        smach.StateMachine.add('RESET_ED_TARGET4',
                               states.ResetED(robot),
                               transitions={'done': 'GOTO_TARGET4_BACKUP'})

        smach.StateMachine.add('GOTO_TARGET4_BACKUP',
                               states.NavigateToWaypoint(robot,
                                                         EntityByIdDesignator(robot, id=challenge_knowledge.target4),
                                                         challenge_knowledge.target4_radius2, look_at_designator=False),
                               transitions={'arrived': 'SAY_TARGET4_REACHED',
                                            'unreachable': 'TIMEOUT4',
                                            'goal_not_defined': 'TIMEOUT4'})

        smach.StateMachine.add('TIMEOUT4',
                               checkTimeOut(robot, challenge_knowledge.time_out_seconds),
                               transitions={'not_yet': 'GOTO_TARGET4', 'time_out': 'SAY_TARGET4_FAILED'})

        # Should we mention that we failed???
        smach.StateMachine.add('SAY_TARGET4_FAILED',
                               states.Say(robot, ["I am not able to reach target 4",
                                                  "I cannot reach target 4",
                                                  "Target 4 is unreachable"], block=True),
                               transitions={'spoken': 'SAY_GOTO_STARTING_POINT'})

        smach.StateMachine.add('SAY_GOTO_STARTING_POINT',
                               states.Say(robot, ["I will go back to the starting point",
                                                  "I will now go to the starting point",
                                                  "Lets go back to where I started",
                                                  "Going to my position at start"], block=False),
                               transitions={'spoken': 'GOTO_STARTING_POINT'})

        ######################################################################################################################################################
        #
        #                                                       In the END
        #
        ######################################################################################################################################################

        smach.StateMachine.add('GOTO_STARTING_POINT',
                                states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.starting_point), challenge_knowledge.starting_point_radius1),
                                transitions={   'arrived'           :   'SAY_STARTING_POINT_REACHED',
                                                'unreachable'       :   'RESET_ED_STARTING_POINT',
                                                'goal_not_defined'  :   'RESET_ED_STARTING_POINT'})

        smach.StateMachine.add( 'SAY_STARTING_POINT_REACHED',
                                states.Say(robot, ["Reached the starting point",
                                                    "I have arrived at the starting point",
                                                    "I am now at the starting point",
                                                    "I am back at where I started"], block=True),
                                transitions={   'spoken'            :   'AT_END'})

        smach.StateMachine.add('RESET_ED_STARTING_POINT',
                                states.ResetED(robot),
                                transitions={   'done'              :   'GOTO_STARTING_POINT_BACKUP'})

        smach.StateMachine.add('GOTO_STARTING_POINT_BACKUP',
                                states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.starting_point), challenge_knowledge.starting_point_radius2),
                                transitions={   'arrived'           :   'SAY_STARTING_POINT_REACHED',
                                                'unreachable'       :   'TIMEOUT_STARTING_POINT',
                                                'goal_not_defined'  :   'TIMEOUT_STARTING_POINT'})

        smach.StateMachine.add( 'TIMEOUT_STARTING_POINT',
                                checkTimeOut(robot, challenge_knowledge.time_out_seconds),
                                transitions={'not_yet': 'GOTO_STARTING_POINT', 'time_out': 'SAY_STARTING_POINT_FAILED'})

        # Should we mention that we failed???
        smach.StateMachine.add( 'SAY_STARTING_POINT_FAILED',
                                states.Say(robot, ["I am not able to reach the starting point",
                                                    "I cannot reach my home position",
                                                    "The starting point is unreachable"], block=True),
                                transitions={   'spoken'            :   'AT_END'})

        smach.StateMachine.add('AT_END',
                               states.Say(robot, "Goodbye"),
                               transitions={'spoken': 'START_DEMO'})

    analyse_designators(sm, "demo_ED")
    return sm


############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('demo_ED')

    states.util.startup(setup_statemachine, challenge_name="demo_ED")
