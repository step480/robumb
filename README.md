# robump
Train Roboy to make a fistbump whenever Kinect detects a fist

## Inspiration
As the goal for Roboy is to have the most human robot possible, it should be able to greet like us and respond when other people greet him in certain ways.  it's only natural that he should be a cool dude, greeting his bros with an appropriate fistbump.
## What it does
Greets someone when the person is standing in front of Roboy. The greeting gesture is made using  roboy’s arms, hands and a facial expression. 
Kinect detects the movements, filter outs some basic movements and sends the detected greeting movement to Roboy which then responds using its hands and face.
Recognizes when someone wants to greet Roboy with a fist bump and triggers Roboy’s fist bumb motion. 

## How we built it
- Gesture recognition: Kinect 2 for sensing, Kinect SDK for training your own gestures, C#-program to send a trigger to our web server (necessary because Kinect runs on Windows, ROS runs on Ubuntu)
- Signal transmitting: http-trigger from the kinect to python-flask-server (`x.x.x.x:5000/bump`)
- Flask server creates ROS node “fistbump_client_py”

 Kinect looks actively for hand movements and interprets the gestures in real-time. Using Kinect SDK  we wrote a “relay” with  C# which sends the commands to ROS Kinetic running on a python server on a  different machine. The ROS on our system establishes a connection with Roboy and sends the movement command to the corresponding Roboy server.

## Challenges we ran into
- Setting up ROS 
	-on Mac OS X 
- getting a high confidence gesture recognition 
- implementing new custom made gestures 
## Accomplishments that we're proud of
-Making Kinect SDK and ROS work together with Roboy although they both are installed on different machines. 
- Learnt a lot about Roboy’s working, both hardware and software. 

## What we learned
- ROS, Flask and how to communicate with and control Roboy via python script
- How to use and train kinect to detect body gestures
- keeping it simple and working as a team
- difference between
- ROS Master and ROS Node
- topic and action
- git rebase vs git merge
- expectations vs reality 
- Developing for Roboy comes with a lot of colorful strings attached.

## What's next for Brofist
- using roboy’s stereo cameras instead of Kinect
- roboy should identify all the greetings possible
- recognize exact position of the greeter's arm to adjust Roboy's arm movement accordingly
- implement other cool bro greetings/handshakes 
- World dominance wi

 

