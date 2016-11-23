using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using System.IO;
using System.IO.MemoryMappedFiles;

using Microsoft.Kinect;

using MonoBrick.EV3;

namespace RoboticArmControll
{
    class Program
    {
        static int[][] motorAngleBorders = new int[][]{
            new int[]{-1200, 1200},
            new int[]{0, 347},
            new int[]{0, 777},
            new int[]{0, 62}
        };

        static int[] startAngles = new int[]{
            0, -200, -390, 0
        };
        static float[] maxSpeed = new float[]{
            100f, 10f, 10f, 100f
        };

        static int[][] armPosBorders = new int[][]{
            new int[]{-180, 180},
            new int[]{-20, 20},
            new int[]{-60, 60},
            new int[]{0, 1}
        };
        static float[] posToAngle = new float[]{
            58.3f, 8f, 8f, 62f
        };

        static float[][] PIDvalues = new float[][]{
            new float[]{1f, 0.0001f, 0.03f},
            new float[]{1f, 0.0001f, 0.03f},
            new float[]{1f, 0.0001f, 0.3f},
            new float[]{1f, 0f, 0f}
        };
        static float[] errorPrevs = new float[]{
            0f, 0f, 0f, 0f
        };
        static float[] errorSums = new float[]{
            0f, 0f, 0f, 0f
        };

        static int[] targetPoses = new int[] { 0, 0, 0, 0 };
        static int[] currentAngles = new int[] { 0, 0, 0, 0 };
        static float[] error = new float[4];
        static Brick<Sensor, Sensor, Sensor, Sensor> _brick;

        static string port = "com20";

        static bool MotorsOn = false;

        static Mutex mutex;

        static string comPath = "C:/Users/Bán T. Titusz/Robotika/RoboticArmControll/ComFile.txt";

        static void Main(string[] args)
        {
            #region Setup
            Console.WriteLine("{0}\nWelcome!\nPlease make sure that the arm is in correct orientation\nAnd the robot is turned on!\n{0}\n", "=======================================================");
            Console.Write("Connect? (y/n): ");
            if (Console.ReadLine().ToLower() != "y")
            {
                return;
            }
        Connect:
            try
            {
                Console.WriteLine("Connecting to: {0}", port);
                Setup(port);
                /*
                mmf = MemoryMappedFile.CreateNew("RobotArm", 0x8000);
                using (MemoryMappedViewStream stream = mmf.CreateViewStream())
                {
                    BinaryWriter writer = new BinaryWriter(stream);
                    writer.Write("Start");
                    writer.Close();
                }*/
                if (!File.Exists(comPath))
                    File.Create(comPath);
                File.WriteAllLines(comPath, new string[] { "0","0","0", "0" });
                bool mutexCreated;
                mutex = new Mutex(false, "RobotArmMutex", out mutexCreated);
                //mutex.ReleaseMutex();

            }
            catch(Exception ex)
            {
                Console.WriteLine(ex);
                Console.Write("Failed to connect. Reconnect? (y/n): ");
                if (Console.ReadLine().ToLower() != "y")
                {
                    return;
                }
                else
                {
                    goto Connect;
                }
            }
            Console.WriteLine("Connected!");
            #endregion
            bool done = false;

            Motor[] motors = new Motor[]{
                _brick.MotorA,
                _brick.MotorB,
                _brick.MotorC,
                _brick.MotorD
            };

            Parallel.Invoke(
() =>
#region MotorControll
{
    while (!done)
    {
        for (int i = 0; i < 4; i++)
        {
            int currentAngleRaw = motors[i].GetTachoCount();
            currentAngles[i] = currentAngleRaw + startAngles[i];
            float clampedTarget = Clamp(targetPoses[i], armPosBorders[i][0], armPosBorders[i][1]);
            float targetAngle = clampedTarget * posToAngle[i];
            error[i] = targetAngle - currentAngles[i];
            errorSums[i] += error[i];
            float error_d = error[i] - errorPrevs[i];
            float PID = error[i] * PIDvalues[i][0] + errorSums[i] * PIDvalues[i][1] + error_d * PIDvalues[i][2];
            sbyte speed = Convert.ToSByte(Clamp(PID, -maxSpeed[i], maxSpeed[i]));
            if (MotorsOn)
                motors[i].On(speed);
            else
                motors[i].Off();
            errorPrevs[i] = error[i];
        }
    }
},
#endregion
() =>
#region directMotorInterface
{
    while (!done)
    {
        Console.Write("Insert Command: ");
        string rawInput = Console.ReadLine().ToLower();
        done = rawInput == "quit";
        string[] firstSplit = rawInput.Split(',');
        foreach (string s in firstSplit)
        {
            string[] split = s.Split(':');
            if (split.Length == 2)
            {
                int direction = 0;
                int pos = 0;
                if (int.TryParse(split[1], out pos))
                {
                    if (!int.TryParse(split[0], out direction))
                    {
                        direction = StringPosIn(split[0], new string[] { "a", "b", "c", "d" });
                    }
                    if (direction >= 1 && direction <= 4)
                    {
                        targetPoses[direction - 1] = (int)Clamp(pos, armPosBorders[direction - 1][0], armPosBorders[direction - 1][1]);
                        Console.WriteLine("Move motor {0} to pos {1}", new string[] { "a", "b", "c", "d" }[direction - 1], targetPoses[direction - 1]);
                    }
                }
            }
            if (s == "help")
            {
                Console.WriteLine("\ta/b/c/d (motor)\n\tget\n\tkinect\n\tmotor\n\twait\n\tquit");
            }
            if (StringPosIn(split[0], new string[] { "a", "b", "c", "d" }) != -1 && split.Length != 2)
            {
                Console.WriteLine("\t:num (angle)");
            }

            if (s.Contains("get"))
            {

                if (split.Length > 1)
                {
                    if (split[1] == "all")
                    {
                        for (int i = 0; i < 4; i++)
                            Console.WriteLine("{0}: {1}", new string[] { "a", "b", "c", "d" }[i], currentAngles[i] / posToAngle[i]);
                    }
                    else
                    {
                        int motor = -1;
                        if (!int.TryParse(split[1], out motor))
                        {
                            motor = StringPosIn(split[1], new string[] { "a", "b", "c", "d" });
                        }
                        if (motor >= 1 && motor <= 4)
                        {
                            Console.WriteLine("{0}: {1}", new string[] { "a", "b", "c", "d" }[motor - 1], currentAngles[motor - 1]);
                        }
                    }
                }
                else
                {
                    Console.WriteLine("\t:all\n\t:a/b/c/d (motor)");
                }
            }
            if (s.Contains("listen"))
            {
                for (int i = 3; i > 0; i--)
                {
                    Console.WriteLine("Startin in: {0}", i);
                    Thread.Sleep(1000);
                }
                bool stopListen = false;
                while (!stopListen) 
                {
                    mutex.WaitOne();
                    /*
                    using (MemoryMappedViewStream stream = mmf.CreateViewStream())
                    {
                        BinaryReader reader = new BinaryReader(stream);
                        try
                        {
                            string s_phi = reader.ReadString();
                            phi = int.Parse(reader.ReadString().Split(':')[1]) - 180;
                            alpha = int.Parse(reader.ReadString().Split(':')[1]);
                            beta = int.Parse(reader.ReadString().Split(':')[1]);
                            arm = int.Parse(reader.ReadString().Split(':')[1]);
                        }
                        catch { }
                    }*/
                    string[] comData = File.ReadAllLines(comPath);
                    mutex.ReleaseMutex();
                    if(comData[0] == "off")
                    {
                        stopListen = true;
                        Console.WriteLine("Listening stopped");
                    }
                    else
                    {
                        for (int j = 0; j < 4; j++)
                        {
                            targetPoses[j] = int.Parse(comData[j + 1]);
                        }
                        Console.WriteLine("{0}, {1}, {2}, {3}", targetPoses[0], targetPoses[1], targetPoses[2], targetPoses[3]);
                    }
                    
                    /*phi = targetPoses[0];
                    alpha = targetPoses[1];
                    beta = targetPoses[2];
                    arm = targetPoses[3];*/
                    //reader.Close();
                    
                    Thread.Sleep(300);
                    
                }
            }
#endregion
#region GeneralInterface
            if (s.Contains("motor"))
            {
                if (split.Length > 1)
                {
                    if (split[1] == "on")
                    {
                        MotorsOn = true;
                    }
                    else if (split[1] == "off")
                    {
                        MotorsOn = false;
                    }
                    Console.WriteLine("Motors: {0}", MotorsOn ? "On" : "Off");
                }
                else
                {
                    Console.WriteLine("\t:on\n\t:off");
                }

            }
            if (s.Contains("wait"))
            {
                if (split.Length > 1)
                {
                    int time = -1;
                    if (split[1] == "all")
                    {
                        Console.WriteLine("Waiting for all motors...");
                        while (Math.Abs(error[0]) > 3 || Math.Abs(error[1]) > 3 || Math.Abs(error[2]) > 3 || Math.Abs(error[3]) > 3)
                        {
                            Thread.Sleep(1);
                        }
                    }
                    else
                    {
                        if (!int.TryParse(split[1], out time))
                        {
                            Thread.Sleep(800);
                            int motor = StringPosIn(split[1], new string[] { "a", "b", "c", "d" });
                            if (motor >= 1 && motor <= 4)
                            {
                                Console.WriteLine("Waiting for motor {0}...", new string[] { "a", "b", "c", "d" }[motor - 1]);
                                while (Math.Abs(error[motor - 1]) > 3)
                                {
                                    Thread.Sleep(1);
                                }
                            }
                        }
                        else
                        {
                            Thread.Sleep(time);
                        }
                    }
                }
                else
                {
                    Console.WriteLine("\t:all\n\t:a/b/c/d (motor)\n\t:num (time(milis))");
                }
            }
        }
    }
}
            #endregion
);
            Console.WriteLine("Quitting...");
            Quit();
            Console.ReadKey();
        }

        static void Setup(string port)
        {
            _brick = new Brick<Sensor, Sensor, Sensor, Sensor>(port);
            _brick.Connection.Open();
            _brick.MotorA.ResetTacho();
            _brick.MotorB.ResetTacho();
            _brick.MotorC.ResetTacho();
            _brick.MotorD.ResetTacho();
        }

        static void Quit()
        {
            _brick.MotorA.Off();
            _brick.MotorB.Off();
            _brick.MotorC.Off();
            _brick.MotorD.Off();
            _brick.Connection.Close();
        }

        static float Clamp(float v, float min, float max)
        {
            if (v < min)
                return min;
            else if (v > max)
                return max;
            else
                return v;
        }
        static int StringPosIn(string t, string[] a)
        {
            for (int i = 0; i < a.Length; i++)
            {
                if (a[i] == t)
                    return i + 1;
            }
            return -1;
        }
    }
}