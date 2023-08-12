package frc.robot;

public class RobotMap {

  // order of subsystems (and adjacent classes) shall be:
  // controllers, drivetrain, arm, intake, collector, charger (if it exists),
  // vision, leds

  public static final class mapControllers {

    public static final int DRIVER_USB = 0;
    public static final int OPERATOR_USB = 1;
    public static final int SWITCHBOARD_USB = 2;
    public static final int NUMPAD_USB = 3;
  }

  public static final class mapDrivetrain {

    public static final int FRONT_LEFT_DRIVE_CAN = 0;
    public static final int FRONT_LEFT_STEER_CAN = 1;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_CAN = 0;

    public static final int FRONT_RIGHT_DRIVE_CAN = 2;
    public static final int FRONT_RIGHT_STEER_CAN = 3;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_CAN = 1;

    public static final int BACK_LEFT_DRIVE_CAN = 4;
    public static final int BACK_LEFT_STEER_CAN = 5;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_CAN = 2;

    public static final int BACK_RIGHT_DRIVE_CAN = 6;
    public static final int BACK_RIGHT_STEER_CAN = 7;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_CAN = 3;

    public static final String CAN_BUS = "Swerve";
  }

  public static final class mapArm {

    public static final int SHOULDER_CAN = 10;
    public static final int ELBOW_CAN = 11;

    public static final int SHOULDER_ABSOLUTE_ENCODER_DIO = 2;
    public static final int ELBOW_ABSOLUTE_ENCODER_DIO = 3;
  }

  public static final class mapIntake {
    public static final int INTAKE_LEFT_MOTOR_CAN = 20;
    public static final int INTAKE_RIGHT_MOTOR_CAN = 21;
    public static final int INTAKE_LIMIT_SWITCH_DIO = 4;
  }

  public static final class mapCollector {
    public static final int PIVOT_MOTOR_CAN = 30;
    public static final int ROLLER_MOTOR_CAN = 31;

    // public static final int PIVOT_ABSOLUTE_ENCODER_DIO = 1;
  }

  public static final class mapElevator {
    public static final int LEFT_MOTOR_CAN = 40;
    public static final int RIGHT_MOTOR_CAN = 41;
  }

  public static final class mapWrist {
    public static final int WRIST_MOTOR_CAN = 50;
  }

  public static final class mapLEDs {
    public static final int BLINKIN_PWM = 9;
  }
}
