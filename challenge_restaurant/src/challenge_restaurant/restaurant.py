#!/usr/bin/python

# ROS
import smach

# TU/e Robotics
import robot_smach_states as states


# Challenge restaurant
from store_waypoint import StoreWaypoint
from take_orders import TakeOrder, ReciteOrders
from wait_for_customer import WaitForCustomer


class Restaurant(smach.StateMachine):
    """ Main statemachine for the restaurant challenge """
    def __init__(self, robot):
        """ Constructor

        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        kitchen_id = "kitchen"
        kitchen_designator = states.util.designators.ed_designators.EdEntityDesignator(robot=robot,
                                                                                       id=kitchen_id)

        caller_id = "customer"
        caller_designator = states.util.designators.ed_designators.EdEntityDesignator(robot=robot,
                                                                                      id=caller_id)

        orders = {}

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'STORE_KITCHEN',
                                                'abort': 'Aborted'})

            smach.StateMachine.add('STORE_KITCHEN',
                                   StoreWaypoint(robot=robot, location_id=kitchen_id),
                                   transitions={'done': 'WAIT_FOR_CUSTOMER'})

            smach.StateMachine.add('WAIT_FOR_CUSTOMER',
                                   WaitForCustomer(robot, caller_id),
                                   transitions={'succeeded': 'NAVIGATE_TO_CUSTOMER',
                                                'failed': 'Aborted',
                                                'aborted': 'Aborted'})

            smach.StateMachine.add('NAVIGATE_TO_CUSTOMER',
                                   states.NavigateToObserve(robot=robot, entity_designator=caller_designator,
                                                            radius=0.7),
                                   transitions={'arrived': 'TAKE_ORDER',
                                                'unreachable': 'TAKE_ORDER',
                                                'goal_not_defined': 'Aborted'})

            smach.StateMachine.add('TAKE_ORDER',
                                   TakeOrder(robot=robot, location=caller_id, orders=orders),
                                   transitions={'succeeded': 'NAVIGATE_TO_KITCHEN',
                                                'failed': 'Aborted'})

            smach.StateMachine.add('NAVIGATE_TO_KITCHEN',
                                   states.NavigateToWaypoint(robot=robot, waypoint_designator=kitchen_designator,
                                                             radius=0.15),
                                   transitions={'arrived': 'RECITE_ORDER',
                                                'unreachable': 'RECITE_ORDER',
                                                'goal_not_defined': 'RECITE_ORDER'})

            smach.StateMachine.add('RECITE_ORDER',
                                   ReciteOrders(robot=robot, orders=orders),
                                   transitions={'spoken': 'Done'})
