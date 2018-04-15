
#! /usr/bin/env python
from __future__ import print_function
import flask as fl

# Initializes a rospy node so that the SimpleActionClient can
# publish and subscribe over ROS.
import rospy
from std_msgs.msg import String
import actionlib
from roboy_communication_control.msg import PerformMovementAction, PerformMovementGoal
try:
    rospy.init_node('fistbump_client_py')
    #faceTopic = rospy.Publisher(
    #   '/roboy/cognition/face/show_emotion',
    #    String, queue_size=10)
    #handTopic = rospy.Publisher(
    #   '/roboy/middleware/FingerCommand',
    #   String, queue_size=10)
    actionClient = actionlib.SimpleActionClient(
       'shoulder_left_movement_server',
       PerformMovementAction)
    print("Waiting for 'shoulder_left_movement_server'...")
    actionClient.wait_for_server()
    print(" ...done.")
    # print("Setting hand default position...")
    #handTopic.publish(String("index: 0, angles: [0, 0, 0, 0]"))
    #handTopic.publish(String("index: 1, angles: [0, 0, 0, 0]"))
    #handTopic.publish(String("index: 2, angles: [0, 0, 0, 0]"))
    #handTopic.publish(String("index: 3, angles: [0, 0, 0, 0]"))
    #handTopic.publish(String("index: 4, angles: [0, 0, 0, 0]"))
except rospy.ROSInterruptException:
    print("program interrupted before completion", file=sys.stderr)

app = fl.Flask(__name__)

@app.route("/hi", methods=['GET', 'POST'])
def hello():
    print("Helle there!")
    return "Hello there!"

@app.route("/bump", methods=['GET', 'POST'])
def makeFistbumpAndEmotion():
    global actionClient #, handTopic
    #faceTopic.publish(String("emotion: 'shy'"))
    bumpGoal = PerformMovementGoal(action = "shoulder_left_fistbump3")
    actionClient.send_goal(bumpGoal)
    #handTopic.publish(String("index: 0, angles: [40, 40, 40, 40]"))
    #handTopic.publish(String("index: 1, angles: [40, 40, 40, 40]"))
    #handTopic.publish(String("index: 2, angles: [40, 40, 40, 40]"))
    #handTopic.publish(String("index: 3, angles: [40, 40, 40, 40]"))
    #handTopic.publish(String("index: 4, angles: [40, 40, 40, 40]"))
    actionClient.wait_for_result()
    return ', '.join([str(n) for n in client.get_result().sequence])

# ============================================================
app.run(host="0.0.0.0", port=5000)
# ============================================================
