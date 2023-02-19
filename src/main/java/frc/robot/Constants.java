// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int JOYSTICK_PORT = 0;
    public static final int CONTROLLER_PORT = 1;
  }

  public static class DriverPorts {
    public static final int CHASIS_LEFT = 0;
    public static final int CHASIS_RIGHT = 1;

    public static final int ELEVATOR_FRONT = 2;
    public static final int ELEVATOR_REAR = 3;

    public static final int ARM_MAIN = 4;

    public static final int GRIPPER_MAIN = 5;
  }

  public static class SensorPorts {
    public static class Elevator {
      public static final int SWITCH_FRONT_BOTTOM = 0;
      public static final int SWITCH_FRONT_TOP = 1;
      public static final int SWITCH_REAR_BOTTOM = 2;
      public static final int SWITCH_REAR_TOP = 3;

      public static final int ENCODER_SIGNAL_A = 4;
      public static final int ENCODER_SIGNAL_B = 5;
    }

    public static class Arm {
      public static final int SWITCH_BOTTOM = 6;
      public static final int SWITCH_TOP = 7;
    }
  }

  public static class MotorSpeedValues {
    public static final double LOW = 0.35;
    public static final double MEDIUM = 0.6;
    public static final double HIGH = 0.8;
    public static final double MAX = 1;

  }
}
