// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.frcteam3255.components.motors.SN_TalonFX;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotPreferences.prefDrivetrain;

/**
 * Coaxial swerve module with SN_TalonFX's for both driving and steering. Uses
 * CANCoder's for the absolute steering encoders.
 */
public class SN_SwerveModule {

  public int moduleNumber;

  private SN_TalonFX driveMotor;
  private SN_TalonFX steerMotor;

  private CANCoder absoluteEncoder;
  private double absoluteEncoderOffset;

  private TalonFXConfiguration driveConfiguration;
  private TalonFXConfiguration steerConfiguration;

  private double lastAngle;

  /**
   * Create a new SN_SwerveModule.
   * 
   * @param moduleConstants Constants required to create a swerve module
   */
  public SN_SwerveModule(SN_SwerveModuleConstants moduleConstants) {
    moduleNumber = moduleConstants.number;

    driveMotor = new SN_TalonFX(moduleConstants.driveMotorID);
    steerMotor = new SN_TalonFX(moduleConstants.steerMotorID);

    absoluteEncoder = new CANCoder(moduleConstants.absoluteEncoderID);
    absoluteEncoderOffset = moduleConstants.absoluteEncoderOffset;

    driveConfiguration = new TalonFXConfiguration();
    steerConfiguration = new TalonFXConfiguration();

    lastAngle = 0;

    configure();
  }

  /**
   * Configure the drive motor, steer motor, and absolute encoder.
   */
  public void configure() {
    // driveMotor

    driveMotor.configFactoryDefault();

    driveConfiguration.slot0.kF = prefDrivetrain.driveF.getValue();
    driveConfiguration.slot0.kP = prefDrivetrain.driveP.getValue();
    driveConfiguration.slot0.kI = prefDrivetrain.driveI.getValue();
    driveConfiguration.slot0.kD = prefDrivetrain.driveD.getValue();

    driveMotor.configAllSettings(driveConfiguration);

    driveMotor.setNeutralMode(Constants.DRIVE_NEUTRAL_MODE);
    driveMotor.setInverted(Constants.DRIVE_MOTOR_INVERT);

    // steerMotor

    steerMotor.configFactoryDefault();

    steerConfiguration.slot0.kP = prefDrivetrain.steerP.getValue();
    steerConfiguration.slot0.kI = prefDrivetrain.steerI.getValue();
    steerConfiguration.slot0.kD = prefDrivetrain.steerD.getValue();

    steerMotor.configAllSettings(steerConfiguration);

    steerMotor.setNeutralMode(Constants.STEER_NEUTRAL_MODE);
    steerMotor.setInverted(Constants.STEER_MOTOR_INVERT);

    // absoluteEncoder

    absoluteEncoder.configFactoryDefault();
    absoluteEncoder.setPositionToAbsolute();
  }

  /**
   * Set the desired state of the swerve module. The state includes the velocity
   * and angles of the module. The state that the motors are actually set to will
   * not always be the desired state, rather it will optimized to reduce
   * unnecessary steering in favor of reversing the direction of the drive motor.
   * 
   * @param desiredState    Desired velocity and angle of the module
   * @param isDriveOpenLoop Is the drive motor velocity set using open or closed
   *                        loop control
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isDriveOpenLoop) {

    // TODO: implement

  }

  /**
   * Neutral the drive motor output.
   */
  public void neutralDriveOutput() {
    driveMotor.neutralOutput();
  }

  /**
   * Reset the steer motor encoder to match the angle of the absolute encoder.
   */
  public void resetSteerMotorEncoderToAbsolute() {
    double absoluteEncoderCount = SN_Math.degreesToFalcon(
        getAbsoluteEncoder().getDegrees(),
        Constants.STEER_GEAR_RATIO);

    steerMotor.setSelectedSensorPosition(absoluteEncoderCount);
  }

  /**
   * Get the absolute encoder with an offset applied. This angle will match the
   * current angle of the module wheel.
   * 
   * @return Absolute encoder with offset applied
   */
  public Rotation2d getAbsoluteEncoder() {

    double radians = Units.degreesToRadians(absoluteEncoder.getAbsolutePosition());

    radians -= Units.degreesToRadians(absoluteEncoderOffset);

    return Rotation2d.fromRadians(radians);
  }

  /**
   * Reset the drive motor encoder count to 0.
   */
  public void resetDriveEncoderCount() {
    driveMotor.setSelectedSensorPosition(0);
  }

  /**
   * Get the current state of the swerve module. State includes a velocity and
   * angle.
   * 
   * @return State of swerve module
   */
  public SwerveModuleState getState() {

    double velocity = SN_Math.falconToMPS(
        driveMotor.getSelectedSensorVelocity(),
        Constants.WHEEL_CIRCUMFERENCE,
        Constants.DRIVE_GEAR_RATIO);

    Rotation2d angle = Rotation2d.fromDegrees(
        SN_Math.falconToDegrees(
            steerMotor.getSelectedSensorPosition(),
            Constants.STEER_GEAR_RATIO));

    return new SwerveModuleState(velocity, angle);
  }

}
