// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SN_SwerveModule;
import frc.robot.RobotPreferences.prefDrivetrain;

public class Drivetrain extends SubsystemBase {

  private SN_SwerveModule[] modules;

  private AHRS navX;

  private SwerveDrivePoseEstimator poseEstimator;

  private boolean isFieldRelative;

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

    configure();
  }

  public void configure() {
    for (SN_SwerveModule mod : modules) {
      mod.configure();
      mod.resetSteerMotorEncodersToAbsolute();
    }
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

  @Override
  public void periodic() {
  }
}
