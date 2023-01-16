package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public class RobotMap {

  public static final class mapDrivetrain {

    public static final int FRONT_LEFT_DRIVE_CAN = 0;
    public static final int FRONT_LEFT_STEER_CAN = 1;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_CAN = 0;

    public static final int FRONT_RIGHT_DRIVE_CAN = 3;
    public static final int FRONT_RIGHT_STEER_CAN = 4;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_CAN = 1;

    public static final int BACK_LEFT_DRIVE_CAN = 6;
    public static final int BACK_LEFT_STEER_CAN = 7;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_CAN = 2;

    public static final int BACK_RIGHT_DRIVE_CAN = 9;
    public static final int BACK_RIGHT_STEER_CAN = 10;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_CAN = 3;
  }

  public static final class mapControllers {

    public static final int DRIVER = 0;
    public static final int SWITCHBOARD = 2;
    public static final int BLINKIN = 0;
  }

  public static final class mapIntake {
    public static final I2C.Port COLOR_SENSOR_I2C = I2C.Port.kMXP;
  }
}
