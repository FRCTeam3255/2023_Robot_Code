package frc.robot;

import com.frcteam3255.preferences.SN_BooleanPreference;
import com.frcteam3255.preferences.SN_DoublePreference;

public class RobotPreferences {

  public static final boolean useNetworkTables = true;

  // order of subsystems (and adjacent classes) shall be:
  // controllers, drivetrain, arm, intake, collector, charger, vision, leds

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

    public static final SN_DoublePreference teleTransP = new SN_DoublePreference("teleTransP", 0);
    public static final SN_DoublePreference teleTransI = new SN_DoublePreference("teleTransI", 0);
    public static final SN_DoublePreference teleTransD = new SN_DoublePreference("teleTransD", 0);
    // feet per second
    public static final SN_DoublePreference teleTransMaxSpeed = new SN_DoublePreference("teleTransMaxSpeed", 16.3);
    // feet per second per second
    public static final SN_DoublePreference teleTransMaxAccel = new SN_DoublePreference("teleTransMaxAccel", 5);
    // inches
    public static final SN_DoublePreference teleTransTolerance = new SN_DoublePreference("teleTransTolerance", 1);

    public static final SN_DoublePreference teleThetaP = new SN_DoublePreference("teleThetaP", 0);
    public static final SN_DoublePreference teleThetaI = new SN_DoublePreference("teleThetaI", 0);
    public static final SN_DoublePreference teleThetaD = new SN_DoublePreference("teleThetaD", 0);
    // degrees per second
    public static final SN_DoublePreference teleThetaMaxSpeed = new SN_DoublePreference("teleThetaMaxSpeed", 360);
    // degrees per second per second
    public static final SN_DoublePreference teleThetaMaxAccel = new SN_DoublePreference("teleThetaMaxAccel", 360);
    // degrees
    public static final SN_DoublePreference teleThetaTolerance = new SN_DoublePreference("teleThetaTolerance", 2);

    // degrees
    public static final SN_DoublePreference tiltedThreshold = new SN_DoublePreference("tiltedThreshold", 14);

    // feet per second
    public static final SN_DoublePreference dockingSpeed = new SN_DoublePreference("dockingSpeed", 5);
  }

  public static final class prefArm {
    public static final SN_DoublePreference shoulderP = new SN_DoublePreference("shoulderP", 2);
    public static final SN_DoublePreference shoulderI = new SN_DoublePreference("shoulderI", 0);
    public static final SN_DoublePreference shoulderD = new SN_DoublePreference("shoulderD", 0);
    // degrees per second
    public static final SN_DoublePreference shoulderMaxSpeed = new SN_DoublePreference("shoulderMaxSpeed", 180);
    // degrees per second per second
    public static final SN_DoublePreference shoulderMaxAccel = new SN_DoublePreference("shoulderMaxAccel", 235);
    // degrees
    public static final SN_DoublePreference shoulderTolerance = new SN_DoublePreference("shoulderTolerance", 0.01);

    public static final SN_DoublePreference elbowP = new SN_DoublePreference("elbowP", 1.75);
    public static final SN_DoublePreference elbowI = new SN_DoublePreference("elbowI", 0);
    public static final SN_DoublePreference elbowD = new SN_DoublePreference("elbowD", 0);
    // degrees per second
    public static final SN_DoublePreference elbowMaxSpeed = new SN_DoublePreference("elbowMaxSpeed", 180);
    // degrees per second per second
    public static final SN_DoublePreference elbowMaxAccel = new SN_DoublePreference("elbowMaxAccel", 235);
    // degrees
    public static final SN_DoublePreference elbowTolerance = new SN_DoublePreference("elbowTolerance", 0.01);

    public static final SN_DoublePreference shoulderAdjustRange = new SN_DoublePreference("shoulderAdjustRange", 30);
    public static final SN_DoublePreference elbowAdjustRange = new SN_DoublePreference("elbowAdjustRange", 15);

    public static final SN_BooleanPreference shoulderForwardSoftLimit = new SN_BooleanPreference(
        "shoulderForwardSoftLimit", true);
    public static final SN_BooleanPreference shoulderReverseSoftLimit = new SN_BooleanPreference(
        "shoulderReverseSoftLimit", true);
    public static final SN_BooleanPreference elbowForwardSoftLimit = new SN_BooleanPreference(
        "elbowForwardSoftLimit", true);
    public static final SN_BooleanPreference elbowReverseSoftLimit = new SN_BooleanPreference(
        "elbowReverseSoftLimit", true);

    public static final SN_DoublePreference armPresetCollectorShoulderAngle = new SN_DoublePreference(
        "armPresetCollectorShoulderAngle", 0);
    public static final SN_DoublePreference armPresetCollectorElbowAngle = new SN_DoublePreference(
        "armPresetCollectorElbowAngle", 0);

    public static final SN_DoublePreference armPresetStowShoulderAngle = new SN_DoublePreference(
        "armPresetStowShoulderAngle", -90);
    public static final SN_DoublePreference armPresetStowElbowAngle = new SN_DoublePreference(
        "armPresetStowElbowAngle", 70);

    public static final SN_DoublePreference armPresetLowShoulderAngle = new SN_DoublePreference(
        "armPresetLowShoulderAngle", -90);
    public static final SN_DoublePreference armPresetLowElbowAngle = new SN_DoublePreference(
        "armPresetLowElbowAngle", 0);

    public static final SN_DoublePreference armPresetConeShoulderAngle = new SN_DoublePreference(
        "armPresetConeShoulderAngle", 2);
    public static final SN_DoublePreference armPresetConeElbowAngle = new SN_DoublePreference(
        "armPresetConeElbowAngle", 15);

    public static final SN_DoublePreference armPresetMidShoulderAngle = new SN_DoublePreference(
        "armPresetMidShoulderAngle", 0);
    public static final SN_DoublePreference armPresetMidElbowAngle = new SN_DoublePreference(
        "armPresetMidElbowAngle", 0);

    public static final SN_DoublePreference armPresetHighShoulderAngle = new SN_DoublePreference(
        "armPresetHighShoulderAngle", 45);
    public static final SN_DoublePreference armPresetHighElbowAngle = new SN_DoublePreference(
        "armPresetHighElbowAngle", 0);
  }

  public static final class prefIntake {
    public static final SN_DoublePreference colorMatcherConfidence = new SN_DoublePreference("colorMatcherConfidence",
        0.95);
    // TODO: Find what proximity is needed for the sensor
    public static final SN_DoublePreference gamePieceProximity = new SN_DoublePreference("gamePieceProximity", 100);

    public static final SN_DoublePreference intakeIntakeSpeed = new SN_DoublePreference("intakeIntakeSpeed", 0.5);
    public static final SN_DoublePreference intakeHoldSpeed = new SN_DoublePreference("intakeHoldSpeed", 0.05);
    public static final SN_DoublePreference intakeReleaseSpeed = new SN_DoublePreference("intakeReleaseSpeed", -0.5);
    public static final SN_DoublePreference intakeReleaseDelay = new SN_DoublePreference("intakeReleaseDelay", 0.25);
  }

  public static final class prefCollector {

    // TODO: Find PID values
    public static final SN_DoublePreference pivotP = new SN_DoublePreference("pivotP", 0.09);
    public static final SN_DoublePreference pivotI = new SN_DoublePreference("pivotI", 0);
    public static final SN_DoublePreference pivotD = new SN_DoublePreference("pivotD", 0);

    // allowable closed loop error in degrees
    public static final SN_DoublePreference pivotTolerance = new SN_DoublePreference(
        "pivotTolerance", .5);
    public static final SN_DoublePreference pivotMaxSpeed = new SN_DoublePreference(
        "pivotMaxSpeed", 1);

    public static final SN_DoublePreference pivotAngleStartingConfig = new SN_DoublePreference(
        "pivotAngleStartingConfig", 206);
    public static final SN_DoublePreference pivotAngleCubeCollecting = new SN_DoublePreference(
        "pivotAngleCubeCollecting", 28);
    public static final SN_DoublePreference pivotAngleClimb = new SN_DoublePreference(
        "pivotAngleClimb", 0);

    public static final SN_DoublePreference rollerSpeed = new SN_DoublePreference("rollerSpeed", .5);

    public static final SN_DoublePreference rollerToleranceMultiplier = new SN_DoublePreference(
        "rollerToleranceMultiplier", 10);
  }

  public static final class prefCharger {
    public static final SN_DoublePreference chargerSpeed = new SN_DoublePreference("chargerSpeed", 0.25);

  }

  public static final class prefVision {

    public static final SN_DoublePreference measurementStdDevsFeet = new SN_DoublePreference(
        "measurementStdDevsFeet", 1);
    public static final SN_DoublePreference measurementStdDevsDegrees = new SN_DoublePreference(
        "measurementStdDevsDegrees", 10);
  }

}
