// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SN_SwerveModule;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefVision;

public class Drivetrain extends SubsystemBase {

  private SN_SwerveModule[] modules;

  private AHRS navX;

  private SwerveDrivePoseEstimator poseEstimator;

  private boolean isFieldRelative;

  private Field2d field;

  private ProfiledPIDController xPID;
  private ProfiledPIDController yPID;
  private ProfiledPIDController thetaPID;

  public SwerveAutoBuilder swerveAutoBuilder;

  public PathPlannerTrajectory linePath;
  public PathPlannerTrajectory twoConePath;

  public Drivetrain() {

    modules = new SN_SwerveModule[] {
        new SN_SwerveModule(Constants.MODULE_0),
        new SN_SwerveModule(Constants.MODULE_1),
        new SN_SwerveModule(Constants.MODULE_2),
        new SN_SwerveModule(Constants.MODULE_3)
    };

    navX = new AHRS();
    navX.reset();

    poseEstimator = new SwerveDrivePoseEstimator(
        Constants.SWERVE_KINEMATICS,
        navX.getRotation2d(),
        getModulePositions(),
        new Pose2d());

    isFieldRelative = true;

    field = new Field2d();

    xPID = new ProfiledPIDController(
        prefDrivetrain.teleTransP.getValue(),
        prefDrivetrain.teleTransI.getValue(),
        prefDrivetrain.teleTransD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.feetToMeters(prefDrivetrain.teleTransMaxSpeed.getValue()),
            Units.feetToMeters(prefDrivetrain.teleTransMaxAccel.getValue())));

    yPID = new ProfiledPIDController(
        prefDrivetrain.teleTransP.getValue(),
        prefDrivetrain.teleTransI.getValue(),
        prefDrivetrain.teleTransD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.feetToMeters(prefDrivetrain.teleTransMaxSpeed.getValue()),
            Units.feetToMeters(prefDrivetrain.teleTransMaxAccel.getValue())));

    thetaPID = new ProfiledPIDController(
        prefDrivetrain.teleThetaP.getValue(),
        prefDrivetrain.teleThetaI.getValue(),
        prefDrivetrain.teleThetaD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(prefDrivetrain.teleThetaMaxSpeed.getValue()),
            Units.degreesToRadians(prefDrivetrain.teleThetaMaxAccel.getValue())));

    linePath = PathPlanner.loadPath("linePath",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));

    twoConePath = PathPlanner.loadPath("twoConePath",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));

    configure();
  }

  public void configure() {
    for (SN_SwerveModule mod : modules) {
      mod.configure();
    }

    xPID.setPID(
        prefDrivetrain.teleTransP.getValue(),
        prefDrivetrain.teleTransI.getValue(),
        prefDrivetrain.teleTransD.getValue());
    xPID.setConstraints(new TrapezoidProfile.Constraints(
        Units.feetToMeters(prefDrivetrain.teleTransMaxSpeed.getValue()),
        Units.feetToMeters(prefDrivetrain.teleTransMaxAccel.getValue())));
    xPID.setTolerance(Units.inchesToMeters(prefDrivetrain.teleTransTolerance.getValue()));
    xPID.reset(getPose().getX());

    yPID.setPID(
        prefDrivetrain.teleTransP.getValue(),
        prefDrivetrain.teleTransI.getValue(),
        prefDrivetrain.teleTransD.getValue());
    yPID.setConstraints(new TrapezoidProfile.Constraints(
        Units.feetToMeters(prefDrivetrain.teleTransMaxSpeed.getValue()),
        Units.feetToMeters(prefDrivetrain.teleTransMaxAccel.getValue())));
    yPID.setTolerance(Units.inchesToMeters(prefDrivetrain.teleThetaTolerance.getValue()));
    yPID.reset(getPose().getY());

    thetaPID.setPID(
        prefDrivetrain.teleThetaP.getValue(),
        prefDrivetrain.teleThetaI.getValue(),
        prefDrivetrain.teleThetaD.getValue());
    thetaPID.setConstraints(new TrapezoidProfile.Constraints(
        Units.degreesToRadians(prefDrivetrain.teleThetaMaxSpeed.getValue()),
        Units.degreesToRadians(prefDrivetrain.teleThetaMaxAccel.getValue())));
    thetaPID.setTolerance(Units.inchesToMeters(prefDrivetrain.teleThetaTolerance.getValue()));
    thetaPID.reset(getPose().getRotation().getRadians());
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);

    // (i think) since the drive motor inversions takes a meanful amount of time, it
    // eats the instruction to reset the encoder counts. so we just wait a second
    // after inverting the modules to reset the steer motor encoders to absolute
    Timer.delay(1.0);
    for (SN_SwerveModule mod : modules) {
      mod.resetSteerMotorEncodersToAbsolute();
    }

    swerveAutoBuilder = new SwerveAutoBuilder(
        this::getPose,
        this::resetPose,
        Constants.SWERVE_KINEMATICS,
        new PIDConstants(
            prefDrivetrain.autoTransP.getValue(),
            prefDrivetrain.autoTransI.getValue(),
            prefDrivetrain.autoTransD.getValue()),
        new PIDConstants(
            prefDrivetrain.autoThetaP.getValue(),
            prefDrivetrain.autoThetaI.getValue(),
            prefDrivetrain.autoThetaD.getValue()),
        this::setModuleStates,
        new HashMap<>(),
        false,
        this);
  }

  /**
   * Drive the drivetrain to a specified position in meters.
   * 
   * @param position Desired position in meters
   */
  public void driveToPosition(Pose2d position) {

    // tell the x and y PID controllers the goal position.
    xPID.setGoal(position.getX());
    yPID.setGoal(position.getY());

    // create a velocity Pose2d with the calculated x and y positions, and the
    // positional rotation.
    Pose2d velocity = new Pose2d(
        xPID.calculate(getPose().getX()),
        yPID.calculate(getPose().getY()),
        position.getRotation());

    // pass the velocity Pose2d to driveAlignAngle(), which will close the loop for
    // rotation and pass the translational values to drive().
    driveAlignAngle(velocity);
  }

  /**
   * Drive the drivetrain with positional absolute heading control.
   * 
   * @param velocity Desired translational velocity in meters per second, and
   *                 desired absolute rotational position
   */
  public void driveAlignAngle(Pose2d velocity) {

    // tell the theta PID controller the goal rotation.
    thetaPID.setGoal(velocity.getRotation().getRadians());

    // calculate the angle setpoint based off where we are now.
    // note that this will not just be the rotation we passed in, it will be some
    // position inbetween.
    double angleSetpoint = thetaPID.calculate(getPose().getRotation().getRadians());

    // create a new velocity Pose2d with the same translation as the on that was
    // passed in, but with the output of the theta PID controller for rotation.
    Pose2d newVelocity = new Pose2d(velocity.getTranslation(), Rotation2d.fromRadians(angleSetpoint));

    // pass the new velocity to the normal drive command
    drive(newVelocity);
  }

  /**
   * Drive the drivetrain
   * 
   * @param velocity Desired translational and rotational velocity in meters per
   *                 second and radians per second
   */
  public void drive(Pose2d velocity) {

    ChassisSpeeds chassisSpeeds;

    if (isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          velocity.getX(),
          velocity.getY(),
          velocity.getRotation().getRadians(),
          getPose().getRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(
          velocity.getX(),
          velocity.getY(),
          velocity.getRotation().getRadians());
    }

    SwerveModuleState[] desiredStates = Constants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(desiredStates);

  }

  /**
   * Set each module state
   * 
   * @param desiredStates Array of desired states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    // desaturateWheelSpeeds() mutates the given array
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_MODULE_SPEED);

    for (SN_SwerveModule mod : modules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], prefDrivetrain.isDriveOpenLoop.getValue());
    }
  }

  public void neutralDriveOutputs() {
    for (SN_SwerveModule mod : modules) {
      mod.neutralDriveOutput();
    }
  }

  /**
   * Set the drive method to use field relative drive controls
   */
  public void setFieldRelative() {
    isFieldRelative = true;
  }

  /**
   * Set the drive method to use robot relative drive controls
   */
  public void setRobotRelative() {
    isFieldRelative = false;
  }

  public void resetPID() {
    xPID.reset(getPose().getX());
    yPID.reset(getPose().getY());
    thetaPID.reset(getPose().getRotation().getRadians());
  }

  /**
   * Get the current estimated position of the drivetrain.
   * 
   * @return Position of drivetrain
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Get an array of each module position. A position consists of a distance and
   * angle.
   * 
   * @return Array of swerve module positions
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SN_SwerveModule mod : modules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }

    return positions;
  }

  /**
   * Updates the pose estimator with the current robot uptime, the gyro yaw, and
   * each swerve module position.
   * <p>
   * This method MUST be called every loop (or else pose estimator breaks)
   */
  public void updatePoseEstimator() {
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        navX.getRotation2d(),
        getModulePositions());

  }

  /**
   * Adds a new vision measurement to the pose estimator.
   *
   * @param visionMeasurement The pose measurement from the vision system
   * @param timestampSeconds  The timestamp of the measurement in seconds
   */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(
        visionMeasurement,
        timestampSeconds,
        VecBuilder.fill(
            Units.feetToMeters(prefVision.measurementStdDevsFeet.getValue()),
            Units.feetToMeters(prefVision.measurementStdDevsFeet.getValue()),
            Units.degreesToRadians(prefVision.measurementStdDevsDegrees.getValue())));
  }

  /**
   * Reset pose estimator to given position
   * 
   * @param pose Position to reset to
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
        navX.getRotation2d(),
        getModulePositions(),
        pose);
  }

  public boolean isTiltedForward() {
    return navX.getRoll() > prefDrivetrain.tiltedThreshold.getValue();
  }

  public boolean isTiltedBackwards() {
    return navX.getRoll() < -prefDrivetrain.tiltedThreshold.getValue();
  }

  @Override
  public void periodic() {

    updatePoseEstimator();

    SmartDashboard.putBoolean("Drivetrain Field Relative", isFieldRelative);

    if (Constants.OUTPUT_DEBUG_VALUES) {

      SmartDashboard.putNumber("Drivetrain Pose X", Units.feetToMeters(getPose().getX()));
      SmartDashboard.putNumber("Drivetrain Pose Y", Units.feetToMeters(getPose().getY()));
      SmartDashboard.putNumber("Drivetrain Pose Rotation", getPose().getRotation().getDegrees());

      SmartDashboard.putBoolean("is Tilted Fowards", isTiltedForward());
      SmartDashboard.putBoolean("is Tilted Backwards", isTiltedBackwards());

      SmartDashboard.putNumber("Drivetrain Yaw", navX.getRotation2d().getDegrees());

      field.setRobotPose(getPose());
      SmartDashboard.putData(field);

      for (SN_SwerveModule mod : modules) {
        SmartDashboard.putNumber("Module " + mod.moduleNumber + " Speed",
            Units.metersToFeet(mod.getState().speedMetersPerSecond));
        SmartDashboard.putNumber("Module " + mod.moduleNumber + " Distance",
            Units.metersToFeet(mod.getPosition().distanceMeters));
        SmartDashboard.putNumber("Module " + mod.moduleNumber + " Angle",
            mod.getState().angle.getDegrees());
        SmartDashboard.putNumber("Module " + mod.moduleNumber + " Absolute Encoder Angle",
            mod.getAbsoluteEncoder().getDegrees());
        SmartDashboard.putNumber("Module " + mod.moduleNumber + " Raw Absolute Encoder Angle",
            mod.getRawAbsoluteEncoder());
        SmartDashboard.putNumber("Module " + mod.moduleNumber + " Drive Output Percent",
            mod.getDriveMotorOutputPercent());
      }
    }
  }
}
