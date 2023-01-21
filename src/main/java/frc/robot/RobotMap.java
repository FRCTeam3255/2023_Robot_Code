package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public class RobotMap {

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

  public static final class mapChargerTreads {

    public static final int LEFT_MOTOR_CAN = 0;
    public static final int RIGHT_MOTOR_CAN = 0;
  }

  public static final class mapControllers {

    public static final int DRIVER_USB = 0;
    public static final int SWITCHBOARD_USB = 2;
    public static final int BLINKIN_PWM = 0;
  }

  public static final class mapIntake {
    public static final I2C.Port COLOR_SENSOR_I2C = I2C.Port.kMXP;
    public static final int LEFT_MOTOR_CAN = 11;
    public static final int RIGHT_MOTOR_CAN = 12;
  }
}
