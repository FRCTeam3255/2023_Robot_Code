package frc.robot;

import com.frcteam3255.preferences.SN_BooleanPreference;
import com.frcteam3255.preferences.SN_DoublePreference;

public class RobotPreferences {
  public static final class prefDrivetrain {

    public static final SN_DoublePreference driveF = new SN_DoublePreference("driveF", 0.045);
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0.1);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0.0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 1.0);

    public static final SN_DoublePreference steerP = new SN_DoublePreference("steerP", 0.6);
    public static final SN_DoublePreference steerI = new SN_DoublePreference("steerI", 0.0);
    public static final SN_DoublePreference steerD = new SN_DoublePreference("steerD", 6.0);

    // percent of max module speed that is required for the module to steer
    // (a stopped wheel shouldn't steer)
    public static final SN_DoublePreference percentToSteer = new SN_DoublePreference("percentToSteer", 0.01);

    public static final SN_BooleanPreference isDriveOpenLoop = new SN_BooleanPreference("isDriveOpenLoop", false);

  }

  public static final class prefVision {

  }
}
