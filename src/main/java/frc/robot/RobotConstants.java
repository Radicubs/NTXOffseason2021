package frc.robot;

public class RobotConstants {
  // FALCON FX MOTOR CONTROLLERS
  public static final int LEFT_FALCON_FRONT = 2;
  public static final int LEFT_FALCON_BACK = 4;

  public static final int RIGHT_FALCON_FRONT = 3;
  public static final int RIGHT_FALCON_BACK = 5;

  // SHOOTER MOTORS
  public static final int SHOOTER_LEFT = 18;
  public static final int SHOOTER_RIGHT = 19;

  // SPARK MAX MOTOR CONTROLLERS
  public static final int ROLLER = 8;
  public static final int INDEX_RIGHT = 11;
  public static final int INDEX_LEFT = 10;
  public static final int ELEVATOR_BACK = 9;
  public static final int ELEVATOR_FRONT = 7;

  //Joystick
  public static final int LEFT_X_AXIS = 0;
  public static final int LEFT_Y_AXIS = 1;
  public static final int LT_AXIS = 2;
  public static final int RT_AXIS = 3;
  public static final int RIGHT_X_AXIS = 4;
  public static final int RIGHT_Y_AXIS = 5;
  public static final int JOYSTICK = 0;

  //Gear Ratio?
  public static final double kGearRatio = 1;

  // PID Constants - need to be changed
  public static final double fl_kP = 0.127;
  public static final double bl_kP = 0.127;
  public static final double fr_kP = 0.127;
  public static final double br_kP = 0.127;

  // Measurements in meters - need to be changed
  public static final double kWheelRadius = 0.0875;
  public static final double kEncoderTicksPerRev = 2048;
  public static final double kFrontRight_y = -1;
        public static final double kFrontRight_x = 0.875;
        public static final double kFrontLeft_y = 1;
        public static final double kFrontLeft_x = 0.875;
        public static final double kBackLeft_y = 1;
        public static final double kBackLeft_x = -0.875;
        public static final double kBackRight_y = -1;
        public static final double kBackRight_x = -0.875;
        public static final double kS = 0.495;
        public static final double kV = 2.04;
        public static final double kA = 0.119;
      }
