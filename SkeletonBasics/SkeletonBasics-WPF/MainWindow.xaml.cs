//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using MonoBrick.EV3;
    using System.Threading;
    using System.Threading.Tasks;
    using System.IO.MemoryMappedFiles;
#region Vector3
    struct Vector3
    {
        public float x;
        public float y;
        public float z;
        public Vector3(SkeletonPoint p)
        {
            this.x = p.X;
            this.y = p.Y;
            this.z = p.Z;
        }
        public Vector3(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public Vector3(double x, double y, double z)
        {
            this.x = (float)x;
            this.y = (float)y;
            this.z = (float)z;
        }
        static public Vector3 operator +(Vector3 lhs, Vector3 rhs)
        {
            return new Vector3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
        }
        static public Vector3 operator -(Vector3 lhs, Vector3 rhs)
        {
            return new Vector3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
        }
        static public Vector3 operator *(Vector3 lhs, float rhs)
        {
            return new Vector3(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
        }
        static public Vector3 operator /(Vector3 lhs, float rhs)
        {
            return lhs * (1f / rhs);
        }
        public float length { get { return (float)Math.Sqrt(this.x * this.x + this.y * this.y + this.z * this.z); } }
        public Vector3 normalized { get { return this / this.length; } }
        public override string ToString()
        {
            return string.Format("({0}, {1}, {2})", this.x, this.y, this.z);
        }
        static public bool operator ==(Vector3 lhs, Vector3 rhs)
        {
            return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
        }
        static public bool operator !=(Vector3 lhs, Vector3 rhs)
        {
            return !(lhs == rhs);
        }
        static public Vector3 Zero = new Vector3(0, 0, 0);
        static public Vector3 Error1 = new Vector3(66f, 66f, 66f);
        static public Vector3 Error2 = new Vector3(666f, 666f, 666f);
        public SkeletonPoint AsSkeletonPoint()
        {
            SkeletonPoint s = new SkeletonPoint();
            s.X = this.x;
            s.Y = this.y;
            s.Z = this.z;
            return s;
        }
    }
#endregion

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
#region originalVariables
        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(System.Windows.Media.Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Blue, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;
#endregion
        static float phi = 0;
        static float alpha = 0;
        static float beta = 0;
        static int arm = 0;

        static float rad2deg = 1f / (2f * (float)Math.PI) * 360f;

        static Thread robotThread;

        static bool done = false;
        static bool motorsOn = false;

        static string comPath = "C:/Users/Bán T. Titusz/Robotika/RoboticArmControll/ComFile.txt";

        static Vector3 relativePos;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
            Console.WriteLine("Loaded");
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
            done = true;
            if(robotThread != null)
                robotThread.Abort();
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
#region LongAndBoring
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight, this.trackedBonePen);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft, this.trackedBonePen);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight, this.trackedBonePen);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft, this.trackedBonePen);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight, this.trackedBonePen);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight, this.trackedBonePen);
#endregion
            this.GetHandRelativePos(skeleton, drawingContext, JointType.HipRight, JointType.HandRight);
            this.HandControlls(skeleton, drawingContext, JointType.ShoulderLeft, JointType.HandLeft);
            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;                    
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;                    
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1, Pen drawingPen)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        private void HandControlls(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }
            // Pens
            Pen targetPen = new Pen(Brushes.Green, 6);

            Vector3 shoulder = new Vector3(skeleton.Joints[jointType0].Position);
            Vector3 hand = new Vector3(skeleton.Joints[jointType1].Position);

            relativePos = hand - shoulder;

            if (relativePos.y > 0)
            {
                checkBoxRobotOn.IsChecked = false;
                motorsOn = false;
            }
            this.DrawBone(skeleton, drawingContext, jointType0, jointType1, targetPen);
        }

        private void GetHandRelativePos(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }
            // Pens
            Pen targetPen = new Pen(Brushes.Red, 6);
            //Pen helpingLinePen = new Pen(Brushes.LightYellow, 3);

            Vector3 hip = new Vector3(skeleton.Joints[jointType0].Position);
            Vector3 hand = new Vector3(skeleton.Joints[jointType1].Position);

            float armLength = 0.5f;

            relativePos = hand - hip;
            float dispphi = (float)Math.Atan2(relativePos.x, relativePos.z);
            phi = (float)Math.Atan2(-relativePos.x, -relativePos.z);
            alpha = (float)((Math.PI / 2) - (Math.Asin(relativePos.y / relativePos.length)) - (Math.Acos(relativePos.length / (2 * armLength))));
            beta = (float)(Math.Acos(1 - Math.Pow(relativePos.length, 2d) / (2 * Math.Pow(armLength, 2d))) - (Math.PI));
            Vector3 i = new Vector3(Math.Sin(alpha) * Math.Sin(dispphi) * armLength, Math.Cos(alpha) * armLength, Math.Sin(alpha) * Math.Cos(dispphi) * armLength);
            float epsilon = (float)Math.PI / 2f + beta - alpha;
            Vector3 j = new Vector3(Math.Cos(epsilon) * Math.Sin(dispphi) * armLength, Math.Sin(epsilon) * armLength, Math.Cos(epsilon) * Math.Cos(dispphi) * armLength);

            drawingContext.DrawLine(targetPen, this.SkeletonPointToScreen(hip.AsSkeletonPoint()), this.SkeletonPointToScreen((hip + relativePos).AsSkeletonPoint()));
            drawingContext.DrawLine(targetPen, this.SkeletonPointToScreen(hip.AsSkeletonPoint()), this.SkeletonPointToScreen((hip + i).AsSkeletonPoint()));
            drawingContext.DrawLine(targetPen, this.SkeletonPointToScreen((hip + i).AsSkeletonPoint()), this.SkeletonPointToScreen((hip + i + j).AsSkeletonPoint()));
        }

        /// <summary>
        /// Handles the checking or unchecking of the seated mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }

        private void Connect_Button_Click(object sender, RoutedEventArgs e)
        {
            if (robotThread == null)
            {
                robotThread = new Thread(RobotThread);
                robotThread.Start();
            }
        }
        
        static void RobotThread()
        {
            Random r = new Random();
            while (!done)
            {
                int p = Convert.ToInt16(Math.Round(phi * rad2deg));
                int al = Convert.ToInt16(alpha * rad2deg);
                int b = Convert.ToInt16(beta * rad2deg + 90);
                string state = motorsOn ? "on" : "off";
                Console.WriteLine("{4}: {0}, {1}, {2}, {3}", p, al, b, arm, state);
                //Console.WriteLine("x: {0}, y: {1}, z: {2}, l: {3}", relativePos.x, relativePos.y, relativePos.z, relativePos.length);

                Mutex mutex = Mutex.OpenExisting("RobotArmMutex");
                mutex.WaitOne();
                File.WriteAllLines(comPath, new string[] {state, p.ToString(), al.ToString(), b.ToString(), arm.ToString() });
                mutex.ReleaseMutex();
                if (motorsOn)
                {
                    Thread.Sleep(500);
                }
                else
                {
                    Thread.Sleep(2000);
                }
                /*try
                {
                    using (MemoryMappedFile mmf = MemoryMappedFile.OpenExisting("RobotArm"))
                    {
                        
                        Mutex mutex = Mutex.OpenExisting("RobotArmMutex");
                        mutex.WaitOne();

                        using (MemoryMappedViewStream stream = mmf.CreateViewStream(1, 0))
                        {
                            BinaryWriter writer = new BinaryWriter(stream);
                            writer.Write(string.Format(":{0}:", p + 180));
                            writer.Write(string.Format(":{0}:", al));
                            writer.Write(string.Format(":{0}:", b));
                            writer.Write(string.Format(":{0}:", a));
                            writer.Close();
                            counter++;
                        }
                        mutex.ReleaseMutex();
                    }
                }
                catch (FileNotFoundException)
                {
                    Console.WriteLine("Memory-mapped file does not exist. Run Process A first.");
                }*/
                
            }
        }

        private void Window_KeyDown(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == System.Windows.Input.Key.W)
            {
                arm = 1;
            }
        }

        private void Window_KeyUp(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == System.Windows.Input.Key.W)
            {
                arm = 0;
            }
        }
        private void CheckBox_On(object sender, RoutedEventArgs e)
        {
            motorsOn = true;
            Console.WriteLine("Arm on");
        }
        private void CheckBox_Off(object sender, RoutedEventArgs e)
        {
            motorsOn = false;
            Console.WriteLine("Arm off");
        }
    }
}