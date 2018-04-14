
#! /usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
from roboy_communication_control.msg import PerformMovementAction, PerformMovementGoal
import pdb

def makeFistbump():
    client = actionlib.SimpleActionClient(
        'shoulder_left_movement_server',
        PerformMovementAction)
    # pdb.set_trace()
    print("Waiting for Server")
    client.wait_for_server()
    bumpAction = PerformMovementGoal(action="shoulder_left_gogogo")

    # bumpAction.action_goal = "shoulder_left_gogogo"
    # pdb.set_trace()
    # bumpAction["action"] = "shoulder_left_gogogo"
    client.send_goal(bumpAction)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fistbump_client_py')
        result = makeFistbump()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

