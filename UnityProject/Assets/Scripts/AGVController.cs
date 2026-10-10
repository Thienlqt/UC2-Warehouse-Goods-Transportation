using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

namespace RosSharp.Control
{
    public enum ControlMode { Keyboard, ROS};

    public class AGVController : MonoBehaviour
    {
        public GameObject wheel1;
        public GameObject wheel2;
        public ControlMode mode = ControlMode.ROS;

        private ArticulationBody wA1;
        private ArticulationBody wA2;

        public float maxLinearSpeed = 2; //  m/s
        public float maxRotationalSpeed = 1;//
        public float wheelRadius = 0.033f; //meters
        public float trackWidth = 0.288f; // meters Distance between tyres
        public float forceLimit = 10;
        public float damping = 10;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;
        // Diagnostics are read-only runtime properties; they are not saved into the prefab.
        public int ReceivedRosCommandCount { get; private set; }
        public float SecondsSinceRosCommand => ReceivedRosCommandCount == 0
            ? float.PositiveInfinity : Time.time - lastCmdReceived;
        public bool RosCommandIsFresh => SecondsSinceRosCommand <= ROSTimeout;
        public float LastReceivedLinearSpeed { get; private set; }
        public float LastReceivedAngularSpeed { get; private set; }
        public bool WheelBodiesReady => wA1 != null && wA2 != null;
        public Vector2 RequestedWheelSpeedsDegreesPerSecond { get; private set; }
        public Vector2 TargetWheelSpeedsDegreesPerSecond => new Vector2(
            wA1 != null ? wA1.xDrive.targetVelocity : float.NaN,
            wA2 != null ? wA2.xDrive.targetVelocity : float.NaN);
        public Vector2 MeasuredWheelSpeedsDegreesPerSecond => new Vector2(
            MeasuredWheelSpeed(wA1), MeasuredWheelSpeed(wA2));
        public bool Wheel1IsSleeping => wA1 != null && wA1.IsSleeping();
        public bool Wheel2IsSleeping => wA2 != null && wA2.IsSleeping();

        static float MeasuredWheelSpeed(ArticulationBody joint)
        {
            // Unity drive targets use degrees/s, but jointVelocity uses radians/s.
            return joint != null && joint.jointType == ArticulationJointType.RevoluteJoint
                && joint.dofCount == 1 ? joint.jointVelocity[0] * Mathf.Rad2Deg : float.NaN;
        }

        ROSConnection ros;
        private RotationDirection direction;
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        void Start()
        {
            wA1 = wheel1 != null ? wheel1.GetComponent<ArticulationBody>() : null;
            wA2 = wheel2 != null ? wheel2.GetComponent<ArticulationBody>() : null;
            if (!WheelBodiesReady || wA1 == wA2)
            {
                Debug.LogError("AGVController: assign Wheel 1 and Wheel 2 to two different "
                    + "wheel objects with ArticulationBody components.", this);
                enabled = false;
                return;
            }
            SetParameters(wA1);
            SetParameters(wA2);
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd);
        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            rosLinear = (float)cmdVel.linear.x;
            rosAngular = (float)cmdVel.angular.z;
            LastReceivedLinearSpeed = rosLinear;
            LastReceivedAngularSpeed = rosAngular;
            ReceivedRosCommandCount++;
            lastCmdReceived = Time.time;
        }

        void FixedUpdate()
        {
            if (mode == ControlMode.Keyboard)
            {
                KeyBoardUpdate();
            }
            else if (mode == ControlMode.ROS)
            {
                ROSUpdate();
            }     
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            joint.xDrive = drive;
        }

        private void KeyBoardUpdate()
        {
            float moveDirection = Input.GetAxis("Vertical");
            float inputSpeed;
            float inputRotationSpeed;
            if (moveDirection > 0)
            {
                inputSpeed = maxLinearSpeed;
            }
            else if (moveDirection < 0)
            {
                inputSpeed = maxLinearSpeed * -1;
            }
            else
            {
                inputSpeed = 0;
            }

            float turnDirction = Input.GetAxis("Horizontal");
            if (turnDirction > 0)
            {
                inputRotationSpeed = maxRotationalSpeed;
            }
            else if (turnDirction < 0)
            {
                inputRotationSpeed = maxRotationalSpeed * -1;
            }
            else
            {
                inputRotationSpeed = 0;
            }
            RobotInput(inputSpeed, inputRotationSpeed);
        }


        private void ROSUpdate()
        {
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }
            RobotInput(rosLinear, -rosAngular);
        }

        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            speed = Mathf.Clamp(speed, -maxLinearSpeed, maxLinearSpeed);
            rotSpeed = Mathf.Clamp(rotSpeed, -maxRotationalSpeed, maxRotationalSpeed);
            float wheel1Rotation = (speed / wheelRadius);
            float wheel2Rotation = wheel1Rotation;
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
            if (rotSpeed != 0)
            {
                wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 2)) * Mathf.Rad2Deg;
                wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 2)) * Mathf.Rad2Deg;
            }
            else
            {
                wheel1Rotation *= Mathf.Rad2Deg;
                wheel2Rotation *= Mathf.Rad2Deg;
            }
            RequestedWheelSpeedsDegreesPerSecond = new Vector2(wheel1Rotation, wheel2Rotation);
            SetSpeed(wA1, wheel1Rotation);
            SetSpeed(wA2, wheel2Rotation);
        }
    }
}
