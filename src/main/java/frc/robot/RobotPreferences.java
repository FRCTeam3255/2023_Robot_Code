package frc.robot;

import com.frcteam3255.preferences.SN_BooleanPreference;
import com.frcteam3255.preferences.SN_DoublePreference;

public class RobotPreferences {

  public static final boolean useNetworkTables = false;

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

  }

  public static final class prefIntake {
    public static final SN_DoublePreference colorMatcherConfidence = new SN_DoublePreference("colorMatcherConfidence",
        0.95);
  }

  public static final class prefVision {

    public static final SN_DoublePreference measurementStdDevsFeet = new SN_DoublePreference(
        "measurementStdDevsFeet", 1);
    public static final SN_DoublePreference measurementStdDevsDegrees = new SN_DoublePreference(
        "measurementStdDevsDegrees", 10);
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
    public static final SN_DoublePreference elbowP = new SN_DoublePreference("elbowP", 0.1);
    public static final SN_DoublePreference elbowI = new SN_DoublePreference("elbowI", 0);
    public static final SN_DoublePreference elbowD = new SN_DoublePreference("elbowD", 0);
    public static final SN_DoublePreference elbowMaxSpeed = new SN_DoublePreference("elbowMaxSpeed", .2);
    public static final SN_DoublePreference elbowTolerance = new SN_DoublePreference("elbowTolerance", 3);

    public static final SN_DoublePreference shoulderPreset = new SN_DoublePreference("shoulderPreset", 0);
    public static final SN_DoublePreference elbowPreset = new SN_DoublePreference("elbowPreset", 0);
  }
}
