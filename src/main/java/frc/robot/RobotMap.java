package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public class RobotMap {

  public static final class mapControllers {

    public static final int DRIVER_USB = 0;
    public static final int OPERATOR_USB = 1;
    public static final int SWITCHBOARD_USB = 2;
    public static final int BLINKIN_PWM = 0;
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

  // arm, intake, collector, charger

  public static final class mapArm {

    public static final int SHOULDER_CAN = 10;
    public static final int ELBOW_CAN = 11;

    public static final int SHOULDER_ABSOLUTE_ENCODER_DIO = 31;
    public static final int ELBOW_ABSOLUTE_ENCODER_DIO = 31;
  }

  public static final class mapIntake {
    public static final I2C.Port COLOR_SENSOR_I2C = I2C.Port.kMXP;
    public static final int INTAKE_LEFT_MOTOR_CAN = 20;
    public static final int INTAKE_RIGHT_MOTOR_CAN = 21;

    public static final int LIMIT_SWITCH_DIO = 0;
  }

  public static final class mapCollector {
    public static final int PIVOT_MOTOR_CAN = 30;
    public static final int ROLLER_MOTOR_CAN = 31;

    public static final int PIVOT_ABSOLUTE_ENCODER_DIO = 1;
  }

  public static final class mapChargerTreads {

    public static final int TREADS_LEFT_MOTOR_CAN = 40;
    public static final int TREADS_RIGHT_MOTOR_CAN = 41;
  }

}
