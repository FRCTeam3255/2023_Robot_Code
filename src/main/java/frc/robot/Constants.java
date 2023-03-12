// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotMap.mapDrivetrain;

public final class Constants {

  public static final boolean OUTPUT_DEBUG_VALUES = true;

  // order of subsystems (and adjacent classes) shall be:
  // controllers, drivetrain, arm, intake, collector, charger (if it exists),
  // vision, leds

  public static final class constControllers {
    public static final double DRIVER_LEFT_STICK_X_DEADBAND = 0.05;
    // public static final double DRIVER_LEFT_STICK_Y_DEADBAND = 0.1;
    // public static final double DRIVER_RIGHT_STICK_X_DEADBAND = 0.1;
    // public static final double DRIVER_RIGHT_STICK_Y_DEADBAND = 0.1;
    // public static final double DRIVER_LEFT_TRIGGER_DEADBAND = 0.0;
    // public static final double DRIVER_RIGHT_TRIGGER_DEADBAND = 0.0;

    // public static final double OPERATOR_LEFT_STICK_X_DEADBAND = 0.1;
    // public static final double OPERATOR_LEFT_STICK_Y_DEADBAND = 0.1;
    // public static final double OPERATOR_RIGHT_STICK_X_DEADBAND = 0.1;
    // public static final double OPERATOR_RIGHT_STICK_Y_DEADBAND = 0.1;
    // public static final double OPERATOR_LEFT_TRIGGER_DEADBAND = 0.0;
    // public static final double OPERATOR_RIGHT_TRIGGER_DEADBAND = 0.0;
  }

  // Drivetrain (no subclass)

  // note: these were physically measured center to center of the wheel on a
  // 29"x29" drivetrain with MK4i's. Ideally these are
  public static final double TRACK_WIDTH = Units.inchesToMeters(23.75);
  public static final double WHEELBASE = Units.inchesToMeters(23.75);

  // Swerve Modules

  /*
   * Order of modules:
   * 
   * 0: Front Left
   * 1: Front Right
   * 2: Back Left
   * 3: Back Right
   * 
   * 0 1
   * 2 3
   *
   * Like reading
   * four words
   */

  // tread is 11 inches long
  // 11 / 3.14 = 3.50 inch wheel (w/o tread) diameter
  // tread is 0.3 inches thick
  // 3.5 + 0.3 = 3.8 inch wheel (w/ tread) diameter
  private static final double WHEEL_DIAMETER = Units.inchesToMeters(3.8);
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

  // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
  // L2 gearing, Falcon drive motor
  public static final double DRIVE_GEAR_RATIO = 6.75;
  public static final double STEER_GEAR_RATIO = 150.0 / 7.0;
  public static final double MAX_MODULE_SPEED = Units.feetToMeters(16.3);

  public static final boolean DRIVE_MOTOR_INVERT = false;
  public static final boolean STEER_MOTOR_INVERT = true;

  public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
  public static final NeutralMode STEER_NEUTRAL_MODE = NeutralMode.Coast;

  public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 18.369141;
  public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 67.675781;
  public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = 85.605469;
  public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = 333.984375;

  // module positions follow the WPILib robot coordinate system
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
  public static final SN_SwerveModuleConstants MODULE_0 = new SN_SwerveModuleConstants(
      mapDrivetrain.FRONT_LEFT_DRIVE_CAN,
      mapDrivetrain.FRONT_LEFT_STEER_CAN,
      mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN,
      FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          WHEELBASE / 2.0,
          TRACK_WIDTH / 2.0),
      0);

  public static final SN_SwerveModuleConstants MODULE_1 = new SN_SwerveModuleConstants(
      mapDrivetrain.FRONT_RIGHT_DRIVE_CAN,
      mapDrivetrain.FRONT_RIGHT_STEER_CAN,
      mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN,
      FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          WHEELBASE / 2.0,
          -TRACK_WIDTH / 2.0),
      1);

  public static final SN_SwerveModuleConstants MODULE_2 = new SN_SwerveModuleConstants(
      mapDrivetrain.BACK_LEFT_DRIVE_CAN,
      mapDrivetrain.BACK_LEFT_STEER_CAN,
      mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN,
      BACK_LEFT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          -WHEELBASE / 2.0,
          TRACK_WIDTH / 2.0),
      2);

  public static final SN_SwerveModuleConstants MODULE_3 = new SN_SwerveModuleConstants(
      mapDrivetrain.BACK_RIGHT_DRIVE_CAN,
      mapDrivetrain.BACK_RIGHT_STEER_CAN,
      mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN,
      BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          -WHEELBASE / 2.0,
          -TRACK_WIDTH / 2.0),
      3);

  public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
      MODULE_0.position,
      MODULE_1.position,
      MODULE_2.position,
      MODULE_3.position);

  public static final double PRAC_FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 250.927734;
  public static final double PRAC_FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 204.433594;
  public static final double PRAC_BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = 151.787109;
  public static final double PRAC_BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = 246.005859;

  // module positions follow the WPILib robot coordinate system
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
  public static final SN_SwerveModuleConstants PRAC_MODULE_0 = new SN_SwerveModuleConstants(
      mapDrivetrain.FRONT_LEFT_DRIVE_CAN,
      mapDrivetrain.FRONT_LEFT_STEER_CAN,
      mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN,
      PRAC_FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          WHEELBASE / 2.0,
          TRACK_WIDTH / 2.0),
      0);

  public static final SN_SwerveModuleConstants PRAC_MODULE_1 = new SN_SwerveModuleConstants(
      mapDrivetrain.FRONT_RIGHT_DRIVE_CAN,
      mapDrivetrain.FRONT_RIGHT_STEER_CAN,
      mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN,
      PRAC_FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          WHEELBASE / 2.0,
          -TRACK_WIDTH / 2.0),
      1);

  public static final SN_SwerveModuleConstants PRAC_MODULE_2 = new SN_SwerveModuleConstants(
      mapDrivetrain.BACK_LEFT_DRIVE_CAN,
      mapDrivetrain.BACK_LEFT_STEER_CAN,
      mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN,
      PRAC_BACK_LEFT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          -WHEELBASE / 2.0,
          TRACK_WIDTH / 2.0),
      2);

  public static final SN_SwerveModuleConstants PRAC_MODULE_3 = new SN_SwerveModuleConstants(
      mapDrivetrain.BACK_RIGHT_DRIVE_CAN,
      mapDrivetrain.BACK_RIGHT_STEER_CAN,
      mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN,
      PRAC_BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          -WHEELBASE / 2.0,
          -TRACK_WIDTH / 2.0),
      3);

  public static final SwerveDriveKinematics PRAC_SWERVE_KINEMATICS = new SwerveDriveKinematics(
      PRAC_MODULE_0.position,
      PRAC_MODULE_1.position,
      PRAC_MODULE_2.position,
      PRAC_MODULE_3.position);

  public static final Rotation2d MODULE_0_DEFENSE_ANGLE = Rotation2d.fromDegrees(45);
  public static final Rotation2d MODULE_1_DEFENSE_ANGLE = Rotation2d.fromDegrees(135);
  public static final Rotation2d MODULE_2_DEFENSE_ANGLE = Rotation2d.fromDegrees(135);
  public static final Rotation2d MODULE_3_DEFENSE_ANGLE = Rotation2d.fromDegrees(45);

  // end drivetrain section

  public static final class constArm {
    public static final boolean SHOULDER_MOTOR_INVERT = true;
    public static final boolean ELBOW_MOTOR_INVERT = false;

    public static final double SHOULDER_GEAR_RATIO = 200.0;
    public static final double ELBOW_GEAR_RATIO = 200.0;

    public static final boolean SHOULDER_ABSOLUTE_ENCODER_INVERT = false;
    public static final boolean ELBOW_ABSOLUTE_ENCODER_INVERT = false;

    public static final NeutralMode SHOULDER_MOTOR_BREAK = NeutralMode.Brake;
    public static final NeutralMode ELBOW_MOTOR_BREAK = NeutralMode.Brake;

    // offsets are when both joints are facing to the right (0 degrees on unit
    // circle is at (1, 0))

    // mini-arm offsets
    // public static final double SHOULDER_ABSOLUTE_ENCODER_OFFSET =
    // Units.rotationsToRadians(0.397309);
    // public static final double ELBOW_ABSOLUTE_ENCODER_OFFSET =
    // Units.rotationsToRadians(0.142530);

    public static final double SHOULDER_ABSOLUTE_ENCODER_OFFSET = Units.rotationsToRadians(0.470116);
    public static final double ELBOW_ABSOLUTE_ENCODER_OFFSET = Units.rotationsToRadians(0.823558);

    public static final double PRAC_SHOULDER_ABSOLUTE_ENCODER_OFFSET = Units.rotationsToRadians(0.346228);
    public static final double PRAC_ELBOW_ABSOLUTE_ENCODER_OFFSET = Units.rotationsToRadians(0.127852);

    public static final double SHOULDER_LENGTH = Units.inchesToMeters(30.0);
    public static final double ELBOW_LENGTH = Units.inchesToMeters(34.0);

    public static final double SHOULDER_FORWARD_LIMIT = Units.degreesToRadians(90.0);
    public static final double SHOULDER_REVERSE_LIMIT = Units.degreesToRadians(-135.0);

    public static final double ELBOW_FORWARD_LIMIT = Units.degreesToRadians(70);
    public static final double ELBOW_REVERSE_LIMIT = Units.degreesToRadians(-100);

    public enum ArmState {
      NONE(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
      STOWED(Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(66)),
      FLOOR_INTAKE(Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-15)),
      SHELF_INTAKE(Rotation2d.fromDegrees(-20), Rotation2d.fromDegrees(10)),
      HIGH_CONE_SCORE(Rotation2d.fromDegrees(-11), Rotation2d.fromDegrees(25)),
      HIGH_CONE_SCORE_LOWERED(Rotation2d.fromDegrees(-11), Rotation2d.fromDegrees(5)),
      MID_CONE_SCORE(Rotation2d.fromDegrees(-59), Rotation2d.fromDegrees(40)),
      MID_CONE_SCORE_LOWERED(Rotation2d.fromDegrees(-59), Rotation2d.fromDegrees(20)),
      HIGH_CUBE_SCORE_PLACE(Rotation2d.fromDegrees(37.5), Rotation2d.fromDegrees(-30.5)),
      HIGH_CUBE_SCORE_SHOOT(Rotation2d.fromDegrees(-99), Rotation2d.fromDegrees(32)),
      MID_CUBE_SCORE(Rotation2d.fromDegrees(-95.5), Rotation2d.fromDegrees(22)),
      HYBRID_SCORE(Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(0));

      public Rotation2d shoulderAngle;
      public Rotation2d elbowAngle;

      private ArmState(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
        this.shoulderAngle = shoulderAngle;
        this.elbowAngle = elbowAngle;
      }
    }
  }

  public static final class constIntake {
    public static final boolean LEFT_MOTOR_INVERTED = false;
    public static final boolean RIGHT_MOTOR_INVERTED = !LEFT_MOTOR_INVERTED;

    public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;

    public static final boolean LIMIT_SWITCH_INVERTED = true;
  }

  public static final class constCollector {
    public static final double GEAR_RATIO = 100;

    public static final boolean PIVOT_FORWARD_LIMIT_ENABLE = true;
    public static final boolean PIVOT_REVERSE_LIMIT_ENABLE = true;

    public static final double PIVOT_FORWARD_LIMIT_VALUE = Units.degreesToRadians(195);
    public static final double PIVOT_REVERSE_LIMIT_VALUE = Units.degreesToRadians(0);

    public static final boolean PIVOT_MOTOR_INVERT = false;
    public static final boolean ROLLER_MOTOR_INVERT = true;
    public static final boolean PIVOT_ABSOLUTE_ENCODER_INVERT = true;

    public static final NeutralMode PIVOT_MOTOR_NEUTRAL_MODE = NeutralMode.Brake;
    public static final NeutralMode ROLLER_MOTOR_NEUTRAL_MODE = NeutralMode.Coast;

    public static final double PIVOT_ABSOLUTE_ENCODER_OFFSET = Units.rotationsToRadians(0.981843);
    public static final double PRAC_PIVOT_ABSOLUTE_ENCODER_OFFSET = Units.rotationsToRadians(0.838061);
  }

  public static final class constVision {
    public static final String LIFECAM_PHOTON_NAME = "Microsoft_LifeCam_HD-3000";
    public static final String AR_PHOTON_NAME = "Global_Shutter_Camera";
    public static final String OV_PHOTON_NAME = "Arducam_OV9281_USB_Camera";

    public static final Transform3d ROBOT_TO_OV = new Transform3d(new Translation3d(-0.149225, -0.1666875, 0.46355),
        new Rotation3d(0, 0, 0));
    public static final Transform3d ROBOT_TO_AR = new Transform3d(new Translation3d(-0.219075, 0.1666875, 0.46355),
        new Rotation3d(0, 0, Units.degreesToRadians(180)));
    public static final Transform3d ROBOT_TO_LIFECAM = new Transform3d(new Translation3d(0.4191, -0.1905, 0.6604),
        new Rotation3d(0, 0, 0));
  }

  public static final class constLEDs {
    public static final PatternType HAS_GAME_PIECE_COLOR = SN_Blinkin.PatternType.Green;

    public static final PatternType FAILURE_COLOR = PatternType.Red;

    public static final PatternType DEFAULT_COLOR = PatternType.Black;

    public static final PatternType DEFENSE_MODE_COLOR = PatternType.RainbowOceanPalette;
  }

  public enum GamePiece {
    NONE, CUBE, CONE, HUH
  }
}
