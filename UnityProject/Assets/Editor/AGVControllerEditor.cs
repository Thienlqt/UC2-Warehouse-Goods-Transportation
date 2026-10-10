using System.Globalization;
using RosSharp.Control;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(AGVController))]
public class AGVControllerEditor : Editor
{
    public override bool RequiresConstantRepaint() => Application.isPlaying;

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Navigation diagnostics", EditorStyles.boldLabel);
        if (!Application.isPlaying)
        {
            EditorGUILayout.HelpBox("Enter Play mode and send a 2D Nav Goal to see command "
                + "reception and wheel motion here.", MessageType.Info);
            return;
        }

        var controller = (AGVController)target;
        EditorGUILayout.HelpBox(CommandStatus(controller), MessageType.Info);
        using (new EditorGUI.DisabledScope(true))
        {
            EditorGUILayout.IntField("Received ROS commands", controller.ReceivedRosCommandCount);
            EditorGUILayout.TextField("Last command age (s)", Age(controller));
            EditorGUILayout.FloatField("Last forward command (m/s)", controller.LastReceivedLinearSpeed);
            EditorGUILayout.FloatField("Last turn command (rad/s)", controller.LastReceivedAngularSpeed);
            EditorGUILayout.LabelField("Wheel speeds: X = Wheel 1, Y = Wheel 2; degrees/s",
                EditorStyles.wordWrappedLabel);
            EditorGUILayout.Vector2Field("Requested wheel speeds", controller.RequestedWheelSpeedsDegreesPerSecond);
            EditorGUILayout.Vector2Field("Motor target speeds", controller.TargetWheelSpeedsDegreesPerSecond);
            EditorGUILayout.Vector2Field("Measured wheel speeds", controller.MeasuredWheelSpeedsDegreesPerSecond);
            if (controller.WheelBodiesReady)
            {
                EditorGUILayout.Toggle("Wheel 1 sleeping", controller.Wheel1IsSleeping);
                EditorGUILayout.Toggle("Wheel 2 sleeping", controller.Wheel2IsSleeping);
            }
        }
        if (GUILayout.Button("Copy diagnostics"))
        {
            EditorGUIUtility.systemCopyBuffer = string.Format(CultureInfo.InvariantCulture,
                "AGVController diagnostics\nObject: {0}\nEnabled: {1}\nMode: {2}\nStatus: {3}\n"
                + "Received commands: {4}\nLast command age (s): {5}\n"
                + "Last forward command (m/s): {6:F3}\nLast turn command (rad/s): {7:F3}\n"
                + "Requested wheel speeds (deg/s, Wheel 1 / Wheel 2): {8}\n"
                + "Motor target speeds (deg/s, Wheel 1 / Wheel 2): {9}\n"
                + "Measured wheel speeds (deg/s, Wheel 1 / Wheel 2): {10}\n"
                + "Wheel bodies ready: {11}\nWheel 1 sleeping: {12}\nWheel 2 sleeping: {13}",
                controller.name, controller.enabled, controller.mode, CommandStatus(controller),
                controller.ReceivedRosCommandCount, Age(controller),
                controller.LastReceivedLinearSpeed, controller.LastReceivedAngularSpeed,
                Speeds(controller.RequestedWheelSpeedsDegreesPerSecond),
                Speeds(controller.TargetWheelSpeedsDegreesPerSecond),
                Speeds(controller.MeasuredWheelSpeedsDegreesPerSecond), controller.WheelBodiesReady,
                controller.Wheel1IsSleeping, controller.Wheel2IsSleeping);
        }
    }

    static string CommandStatus(AGVController controller)
    {
        if (!controller.enabled)
            return "Controller disabled: check its checkbox and the Console for errors.";
        if (!controller.WheelBodiesReady)
            return "Wheel bodies unavailable: check the wheel references and Console.";
        if (controller.mode != ControlMode.ROS)
            return "Keyboard mode: the controller is using keyboard input.";
        if (controller.ReceivedRosCommandCount == 0)
            return "Waiting for cmd_vel: no ROS movement command has reached this controller.";
        return controller.RosCommandIsFresh
            ? "Receiving fresh ROS movement commands. Compare wheel targets with measured speeds."
            : "Last ROS command timed out: the controller stops the wheels until a fresh command arrives.";
    }

    static string Age(AGVController controller) => controller.ReceivedRosCommandCount == 0
        ? "No command received" : controller.SecondsSinceRosCommand.ToString("F3", CultureInfo.InvariantCulture);

    static string Speeds(Vector2 value) => string.Format(CultureInfo.InvariantCulture,
        "{0:F3} / {1:F3}", value.x, value.y);
}
