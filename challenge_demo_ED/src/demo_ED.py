#!/usr/bin/python
import rospy
import smach
import sys
import time

from cb_planner_msgs_srvs.msg import PositionConstraint

from robot_smach_states.util.designators import VariableDesignator, EdEntityDesignator, EntityByIdDesignator, \
    analyse_designators
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


class DetermineDoor(smach.State):
    def __init__(self, robot, door_id_designator):
        smach.State.__init__(self, outcomes=["door_found", "preempted"])
        self._robot = robot
        self._door_id_designator = door_id_designator

    def execute(self, userdata):
        door_1_position = self._robot.ed.get_entity(id=challenge_knowledge.reentry_door_1).pose.position
        door_2_position = self._robot.ed.get_entity(id=challenge_knowledge.reentry_door_2).pose.position

        door_1_constraint = PositionConstraint()
        door_1_constraint.constraint = "(x-%f)^2 + (y-%f)^2 < %f^2" % (door_1_position.x, door_1_position.y, 0.7)

        door_2_constraint = PositionConstraint()
        door_2_constraint.constraint = "(x-%f)^2 + (y-%f)^2 < %f^2" % (door_2_position.x, door_2_position.y, 0.7)

        # TODO: make sure that waypoints exist!!!
        # TODO: make sure door id designator is writeable

        while True:
            plan1 = self._robot.base.global_planner.getPlan(door_1_constraint)
            plan2 = self._robot.base.global_planner.getPlan(door_2_constraint)
            print "Plan 1 length: %i" % len(plan1)
            print "Plan 2 length: %i" % len(plan2)
            if len(plan1) < 3:
                self._door_id_designator.writeable.write(challenge_knowledge.target_door_1)
                return "door_found"
            elif len(plan2) < 3:
                self._door_id_designator.writeable.write(challenge_knowledge.target_door_2)
                return "door_found"

            if self.preempt_requested():
                return 'preempted'

            time.sleep(1)


class DetermineObject(smach.State):
    def __init__(self, robot, entity_id, obstacle_radius):
        smach.State.__init__(self, outcomes=["done", "timeout"])
        self._robot = robot

        try:
            pose = robot.ed.get_entity(id=entity_id).pose
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

        self.pc = PositionConstraint(frame="/map",
                                     constraint="(x-%f)^2+(y-%f)^2 < 0.05" % (pose.position.x, pose.position.y))
        self.obstacle_radius = obstacle_radius

    def execute(self, userdata):

        self._robot.speech.speak("Waypoint 2 is occupied, what do we have here", block=False)

        # Make sure you look where you would expect the person to stand
        self._robot.head.look_at_standing_person()
        self._robot.head.wait_for_motion_done()
        time.sleep(1)

        # Check if there is a human blocking the path
        rospy.logerr(
            "ed.detect _persons() method disappeared! This was only calling the face recognition module and we are using a new one now!")
        rospy.logerr("I will return an empty detection list!")
        persons = []

        if not persons:
            persons = []

        rospy.loginfo("Person detection result: %s" % persons)

        block_is_person = False
        for person in persons:
            #            pose_base_link = self._robot.tf_listener.transformPose(target_frame=self._robot.robot_name+'/base_link',
            #                                                                   pose=person.pose)
            #
            #            x = pose_base_link.pose.position.x
            #            y = pose_base_link.pose.position.y

            #            r = self.obstacle_radius  # Distance from the robot's base link in the x-direction
            #            if (x - r)*(x - r) + y*y < r*r:
            block_is_person = True
            break

        # Stop looking at person
        self._robot.head.cancel_goal()

        if block_is_person:
            self._robot.speech.speak("Hi there Human, please step aside")
        else:
            self._robot.speech.speak("Can somebody please remove the non-human object that is blocking waypoint 2?")

        start_time = time.time()
        while not self._robot.base.global_planner.getPlan(self.pc):
            if time.time() - start_time > 30:
                return "timeout"
            time.sleep(1)

        self._robot.speech.speak("Thank you")

        time.sleep(3)

        return "done"


def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:
        # Start challenge via StartChallengeRobust
        smach.StateMachine.add("START_CHALLENGE_ROBUST",
                               states.StartChallengeRobust(robot, challenge_knowledge.starting_point,
                                                           use_entry_points=True, door=False),
                               transitions={"Done": "SAY_GOTO_TARGET1",
                                            "Aborted": "SAY_GOTO_TARGET1",
                                            "Failed": "SAY_GOTO_TARGET1"})

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
                                                         challenge_knowledge.target1_radius1),
                               transitions={'arrived': 'SAY_TARGET1_REACHED',
                                            'unreachable': 'RESET_ED_TARGET1',
                                            'goal_not_defined': 'RESET_ED_TARGET1'})

        smach.StateMachine.add('SAY_TARGET1_REACHED',
                               states.Say(robot, ["Reached target 1",
                                                  "I have arrived at target 1",
                                                  "I am now at target 1"], block=True),
                               transitions={'spoken': 'SAY_GOTO_TARGET3'})

        smach.StateMachine.add('RESET_ED_TARGET1',
                               states.ResetED(robot),
                               transitions={'done': 'GOTO_TARGET1_BACKUP'})

        smach.StateMachine.add('GOTO_TARGET1_BACKUP',
                               states.NavigateToWaypoint(robot,
                                                         EntityByIdDesignator(robot, id=challenge_knowledge.target1),
                                                         challenge_knowledge.target1_radius2),
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
                                                         challenge_knowledge.target2_radius1),
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
                                                         challenge_knowledge.target2_radius2),
                               transitions={'arrived': 'SAY_TARGET2_REACHED',
                                            'unreachable': 'TIMEOUT2',
                                            'goal_not_defined': 'TIMEOUT2'})

        smach.StateMachine.add('TIMEOUT2,
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
                                states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.target3), challenge_knowledge.target3_radius1),
                                transitions={   'arrived'           :   'SAY_TARGET3_REACHED',
                                                'unreachable'       :   'RESET_ED_TARGET3',
                                                'goal_not_defined'  :   'RESET_ED_TARGET3'})

        smach.StateMachine.add( 'SAY_TARGET3_REACHED',
                                states.Say(robot, ["Reached target 3",
                                                    "I have arrived at target 3",
                                                    "I am now at target 3"], block=True),
                                transitions={   'spoken'            :   'SAY_GOTO_STARTING_POINT'})

        smach.StateMachine.add('RESET_ED_TARGET3',
                                states.ResetED(robot),
                                transitions={   'done'              :   'GOTO_TARGET3_BACKUP'})

        smach.StateMachine.add('GOTO_TARGET3_BACKUP',
                                states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.target3), challenge_knowledge.target3_radius2),
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
                                transitions={   'spoken'            :   'SAY_GOTO_STARTING_POINT'})

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

        smach.StateMachine.add('GOTO_TARGET3_BACKUP',
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
                               transitions={'spoken': 'Done'})

    analyse_designators(sm, "demo_ED")
    return sm


############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('demo_ED')

    states.util.startup(setup_statemachine, challenge_name="demo_ED")
