
#! /usr/bin/env python
from __future__ import print_function
import flask as fl
import rospy
import actionlib
##from roboy_communication_control.msg import PerformMovementAction, PerformMovementGoal

app = fl.Flask(__name__)

@app.route("/hi", methods=['GET', 'POST'])
def hello():
    print("Helle there!")
    return "Hello there!"

@app.route("/bump", methods=['GET', 'POST'])
def makeFistbump():
    client = actionlib.SimpleActionClient(
        'shoulder_left_movement_server',
        PerformMovementAction)
    # pdb.set_trace()
    print("Waiting for Server")
    client.wait_for_server()
    bumpGoal = PerformMovementGoal(action = "shoulder_left_gogogo")
    client.send_goal(bumpGoal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        ##rospy.init_node('fistbump_client_py')
        app.run(host="0.0.0.0", port=5000)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
