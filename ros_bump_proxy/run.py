
#! /usr/bin/env python

import rospy
from __future__ import print_function

import actionlib
import roboy_communication_control.msg

def makeFistbump():
    client = actionlib.SimpleActionClient(
        'bump',
        roboy_communication_control.msg.PerformMovementAction)
    client.wait_for_server()
    bumpAction = roboy_communication_control.msg.PerformMovementAction(name="fistbump")
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

