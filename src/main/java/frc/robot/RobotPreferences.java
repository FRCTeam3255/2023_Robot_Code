package frc.robot;

import com.frcteam3255.preferences.SN_BooleanPreference;
import com.frcteam3255.preferences.SN_DoublePreference;

public class RobotPreferences {

  public static final boolean useNetworkTables = true;

  public static final class prefDrivetrain {

    public static final SN_DoublePreference driveF = new SN_DoublePreference("driveF", 0.045);
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0.1);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0.0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 1.0);

    public static final SN_DoublePreference steerP = new SN_DoublePreference("steerP", 0.3);
    public static final SN_DoublePreference steerI = new SN_DoublePreference("steerI", 0.0);
    public static final SN_DoublePreference steerD = new SN_DoublePreference("steerD", 6.0);

    // percent of max module speed that is required for the module to steer
    // (a stopped wheel shouldn't steer)
    public static final SN_DoublePreference percentToSteer = new SN_DoublePreference("percentToSteer", 0.01);

    public static final SN_BooleanPreference isDriveOpenLoop = new SN_BooleanPreference("isDriveOpenLoop", false);

    // max translational speed in feet per second while driving using a controller
    // 16.3 FPS is maximum due to gearing
    public static final SN_DoublePreference driveSpeed = new SN_DoublePreference("driveSpeed", 16.3);

    // max rotational speed in degrees per second while driving using a controller
    // 943.751 DPS is maximum due to gearing and robot size
    public static final SN_DoublePreference turnSpeed = new SN_DoublePreference("turnSpeed", 360);

    // Value to multiply with translation velocity when trigger is all the way held
    // down.
    public static final SN_DoublePreference triggerValue = new SN_DoublePreference("triggerValue", .2);

    // TODO: Create PID values and other values
    public static final SN_DoublePreference autoThetaP = new SN_DoublePreference("autoThetaP", 0.7);
    public static final SN_DoublePreference autoThetaI = new SN_DoublePreference("autoThetaI", 0.0);
    public static final SN_DoublePreference autoThetaD = new SN_DoublePreference("autoThetaD", 0.0);

    public static final SN_DoublePreference autoTransP = new SN_DoublePreference("autoTransP", 2);
    public static final SN_DoublePreference autoTransI = new SN_DoublePreference("autoTransI", 0);
    public static final SN_DoublePreference autoTransD = new SN_DoublePreference("autoTransD", 0);

    public static final SN_DoublePreference autoMaxSpeedFeet = new SN_DoublePreference(
        "autoMaxSpeedFeet", 2.0);

    public static final SN_DoublePreference autoMaxAccelFeet = new SN_DoublePreference(
        "autoMaxAccelFeet", 1.0);
  }

  public static final class prefChargerTreads {
    public static final SN_DoublePreference motorSpeed = new SN_DoublePreference("motorSpeed", 0.25);

  }

  public static final class prefIntake {
    public static final SN_DoublePreference colorMatcherConfidence = new SN_DoublePreference("colorMatcherConfidence",
        0.95);
    // TODO: Find what proximity is needed for the sensor
    public static final SN_DoublePreference gamePieceProximity = new SN_DoublePreference("gamePieceProximity", 100);

    public static final SN_DoublePreference intakeIntakeSpeed = new SN_DoublePreference("intakeIntakeSpeed", 0.5);
    public static final SN_DoublePreference intakeHoldSpeed = new SN_DoublePreference("intakeHoldSpeed", 0.05);
    public static final SN_DoublePreference intakeReleaseSpeed = new SN_DoublePreference("intakeReleaseSpeed", -0.5);
  }

  public static final class prefVision {

    public static final SN_DoublePreference measurementStdDevsFeet = new SN_DoublePreference(
        "measurementStdDevsFeet", 1);
    public static final SN_DoublePreference measurementStdDevsDegrees = new SN_DoublePreference(
        "measurementStdDevsDegrees", 10);
  }

  public static final class prefCollector {

    // TODO: Find PID values
    public static final SN_DoublePreference pivotP = new SN_DoublePreference("pivotP", 0.09);
    public static final SN_DoublePreference pivotI = new SN_DoublePreference("pivotI", 0);
    public static final SN_DoublePreference pivotD = new SN_DoublePreference("pivotD", 0);

    // allowable closed loop error in degrees
    public static final SN_DoublePreference pivotTolerance = new SN_DoublePreference(
        "pivotTolerance", 2);
    public static final SN_DoublePreference pivotMaxSpeed = new SN_DoublePreference(
        "pivotMaxSpeed", 0.25);

    public static final SN_DoublePreference pivotAngleStartingConfig = new SN_DoublePreference(
        "pivotAngleStartingConfig", 0);
    public static final SN_DoublePreference pivotAngleCubeCollecting = new SN_DoublePreference(
        "pivotAngleCubeCollecting", 90);
    public static final SN_DoublePreference pivotAngleClimb = new SN_DoublePreference(
        "pivotAngleClimb", 180);

    public static final SN_DoublePreference rollerSpeed = new SN_DoublePreference("rollerSpeed", 1);
  }

  public static final class prefArm {
    public static final SN_DoublePreference shoulderArbitraryFeedForward = new SN_DoublePreference(
        "shoulderArbitraryFeedForward", 0);
    public static final SN_DoublePreference shoulderP = new SN_DoublePreference("shoulderP", 0.1);
    public static final SN_DoublePreference shoulderI = new SN_DoublePreference("shoulderI", 0);
    public static final SN_DoublePreference shoulderD = new SN_DoublePreference("shoulderD", 0);
    public static final SN_DoublePreference shoulderMaxSpeed = new SN_DoublePreference("shoulderMaxSpeed", .2);
    public static final SN_DoublePreference shoulderTolerance = new SN_DoublePreference("shoulderTolerance", 3);

    public static final SN_DoublePreference elbowArbitraryFeedForward = new SN_DoublePreference(
        "elbowArbitraryFeedForward", 0);
    public static final SN_DoublePreference elbowP = new SN_DoublePreference("elbowP", 0);
    public static final SN_DoublePreference elbowI = new SN_DoublePreference("elbowI", 0);
    public static final SN_DoublePreference elbowD = new SN_DoublePreference("elbowD", 0);
    public static final SN_DoublePreference elbowMaxSpeed = new SN_DoublePreference("elbowMaxSpeed", .2);
    public static final SN_DoublePreference elbowTolerance = new SN_DoublePreference("elbowTolerance", 3);

    public static final SN_DoublePreference shoulderPreset = new SN_DoublePreference("shoulderPreset", 0);
    public static final SN_DoublePreference elbowPreset = new SN_DoublePreference("elbowPreset", 45.0);

    // radius of circle centered on shoulder joint where the arm tip cannot reach
    // (inches)
    public static final SN_DoublePreference armTipDeadzone = new SN_DoublePreference("armTipDeadzone", 2);

    public static final SN_DoublePreference armTipPresetX = new SN_DoublePreference("armTipPresetX", 8);
    public static final SN_DoublePreference armTipPresetY = new SN_DoublePreference("armTipPresetY", 8);
    public static final SN_BooleanPreference shoulderForwardSoftLimit = new SN_BooleanPreference(
        "shoulderForwardSoftLimit", true);
    public static final SN_BooleanPreference shoulderReverseSoftLimit = new SN_BooleanPreference(
        "shoulderReverseSoftLimit", true);
    public static final SN_BooleanPreference elbowForwardSoftLimit = new SN_BooleanPreference(
        "elbowForwardSoftLimit", true);
    public static final SN_BooleanPreference elbowReverseSoftLimit = new SN_BooleanPreference(
        "elbowReverseSoftLimit", true);

    public static final SN_DoublePreference armTipToCollectorX = new SN_DoublePreference("armTipToCollectorX", 15);
    public static final SN_DoublePreference armTipToCollectorY = new SN_DoublePreference("armTipToCollectorY", 2);

    public static final SN_DoublePreference armTipToMidPosX = new SN_DoublePreference("armTipToMidPosX", 4);
    public static final SN_DoublePreference armTipToMidPosY = new SN_DoublePreference("armTipToMidPosY", 15);
  }
}
