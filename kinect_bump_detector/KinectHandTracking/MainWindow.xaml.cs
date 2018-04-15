using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Net.Http;

namespace KinectHandTracking
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Members
        private static readonly HttpClient client = new HttpClient(); //Used to conatct python server. The python server initiates contact to Roboy when it receives a signal from here.
        private string rightHandState = "-"; //Right hand state is saved here.
        private string leftHandState = "-"; //Left hand state is saved here.
        bool sendSignal = true; //Whether to send msg to the server. Initially true. Becomes false immediately after a signal is sent. Becomes true again when the timer runs out.
        bool handClosedDetected = false; //Used to track state transition. Closed -> Open, Closed -> Lasso.

        //These two variables are used to track passage of time in order to allow for multiple signals to be sent to the server while making sure that Roboy is done performing whatever action he's supposed to perform.
        Stopwatch sw = new Stopwatch(); 
        TimeSpan maxTime = TimeSpan.FromMinutes(0.1);

        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        IList<Body> _bodies;

        #endregion

        #region Constructor

        public MainWindow()
        {
            InitializeComponent();
        }

        #endregion

        #region Event handlers

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.GetDefault();

            if (_sensor != null)
            {
                _sensor.Open();

                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body);
                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
                _reader.MultiSourceFrameArrived += HandshakeInitiate; //Added by team.
            }
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            if (_reader != null)
            {
                _reader.Dispose();
            }

            if (_sensor != null)
            {
                _sensor.Close();
            }
        }

        //Send signal to python server that the Kinect has identified a specific hand gesture.
        private async void msgToPythonServer()
        {
            var values = new Dictionary<string, string>
            {
               { "bla", "initiate handshake" }
            };

            var content = new FormUrlEncodedContent(values);

            //var response = await client.PostAsync("http://10.183.58.154:5000/hi", content);

            //var responseString = await response.Content.ReadAsStringAsync();
        }

        //Returns true if the timer has run out.
        private bool checkWaitOver()
        {
            bool timeUp = false;
            if(sw.Elapsed >= maxTime)
            {
                timeUp = true;
            }
            return timeUp;
        }

        //Detect a specific hand gesture (here Close -> Open, Close -> Lasso).
        //Once the gesture is detected, a signal is sent to a python server which initiates Roboy's reaction.
        //A timer is also started when the signal is sent. Once the timer runs out, a new signal can be sent if a gesture is detected again.
        //This method is called every frame if the kinect camera.
        void HandshakeInitiate(object sender, MultiSourceFrameArrivedEventArgs e)
        {                       
            if (sendSignal)
            {
                if(rightHandState == "Closed")
                {                    
                    handClosedDetected = true;
                }
                else if ((rightHandState == "Open" || rightHandState == "Lasso") && handClosedDetected)
                {
                    //call server
                    Debug.WriteLine("Go Roboy");
                    msgToPythonServer();
                    sendSignal = false;
                    sw.Start();
                }
            }
            bool timeUp = checkWaitOver();
            if (timeUp)
            {
                Debug.WriteLine("Time's up!!");
                sendSignal = true;
                sw.Reset();
            }                
        }


        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();

            // Color
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    camera.Source = frame.ToBitmap();
                }
            }

            // Body
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    canvas.Children.Clear();

                    _bodies = new Body[frame.BodyFrameSource.BodyCount];

                    frame.GetAndRefreshBodyData(_bodies);

                    foreach (var body in _bodies)
                    {
                        if (body != null)
                        {
                            if (body.IsTracked)
                            {
                                // Find the joints
                                Joint handRight = body.Joints[JointType.HandRight];
                                Joint thumbRight = body.Joints[JointType.ThumbRight];

                                Joint handLeft = body.Joints[JointType.HandLeft];
                                Joint thumbLeft = body.Joints[JointType.ThumbLeft];

                                // Draw hands and thumbs
                                canvas.DrawHand(handRight, _sensor.CoordinateMapper);
                                canvas.DrawHand(handLeft, _sensor.CoordinateMapper);
                                canvas.DrawThumb(thumbRight, _sensor.CoordinateMapper);
                                canvas.DrawThumb(thumbLeft, _sensor.CoordinateMapper);

                                // Find the hand states
                                //string rightHandState = "-";
                                //string leftHandState = "-";

                                switch (body.HandRightState)
                                {
                                    case HandState.Open:
                                        rightHandState = "Open";
                                        break;
                                    case HandState.Closed:
                                        rightHandState = "Closed";
                                        break;
                                    case HandState.Lasso:
                                        rightHandState = "Lasso";
                                        break;
                                    case HandState.Unknown:
                                        rightHandState = "Unknown...";
                                        break;
                                    case HandState.NotTracked:
                                        rightHandState = "Not tracked";
                                        break;
                                    default:
                                        break;
                                }

                                switch (body.HandLeftState)
                                {
                                    case HandState.Open:
                                        leftHandState = "Open";
                                        break;
                                    case HandState.Closed:
                                        leftHandState = "Closed";
                                        break;
                                    case HandState.Lasso:
                                        leftHandState = "Lasso";
                                        break;
                                    case HandState.Unknown:
                                        leftHandState = "Unknown...";
                                        break;
                                    case HandState.NotTracked:
                                        leftHandState = "Not tracked";
                                        break;
                                    default:
                                        break;
                                }

                                tblRightHandState.Text = rightHandState;
                                tblLeftHandState.Text = leftHandState;
                            }
                        }
                    }
                }
            }
        }

        #endregion
    }
}
