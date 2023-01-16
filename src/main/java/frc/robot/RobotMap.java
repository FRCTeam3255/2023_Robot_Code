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

    public static final int DRIVER_USB = 0;
    public static final int SWITCHBOARD_USB = 2;
    public static final int BLINKIN_PWM = 0;
  }

  public static final class mapIntake {
    public static final I2C.Port COLOR_SENSOR_I2C = I2C.Port.kMXP;
  }

  public static final class mapArm {

    // TODO: Update CAN ports for arm motors
    public static final int SHOULDER_CAN = 0;
    public static final int ELBOW_CAN = 1;

    // TODO: Update CAN ports for arm encoders
    public static final int SHOULDER_ABSOLUTE_ENCODER_CAN = 0;
    public static final int ELBOW_ABSOLUTE_ENCODER_CAN = 1;

    // TODO: update position degrees
    public static final double SHOULDER_POSITION_DEGRESS_RETRACTED = 0;
    public static final double SHOULDER_POSITION_DEGRESS_EXTENDED = 90;

    // TODO: update position degreesd
    public static final double ELBOW_POSITION_DEGRESS_RETRACTED = 0;
    public static final double ELBOW_POSITION_DEGRESS_EXTENDED = 90;

    // Update conversion factor for robot
    public static final double ENCODER_CONVERSION_FACTOR = 1.8;
    public static final int DISTANCE_PER_ROTATION = 360;
  }
}
