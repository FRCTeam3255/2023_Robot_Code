// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.frcteam3255.utils.CTREModuleState;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;

/**
 * Coaxial swerve module with SN_TalonFX's for both driving and steering. Uses
 * CANCoder's for the absolute steering encoders.
 */
public class SN_SwerveModule {

  public int moduleNumber;

  private TalonFX driveMotor;
  private TalonFX steerMotor;

  private CANCoder absoluteEncoder;
  private double absoluteEncoderOffset;

  private TalonFXConfiguration driveConfiguration;
  private TalonFXConfiguration steerConfiguration;

  private SupplyCurrentLimitConfiguration driveCurrentLimit;
  private SupplyCurrentLimitConfiguration steerCurrentLimit;

  private double lastAngle;

  /**
   * Create a new SN_SwerveModule.
   * 
   * @param moduleConstants Constants required to create a swerve module
   */
  public SN_SwerveModule(SN_SwerveModuleConstants moduleConstants) {
    moduleNumber = moduleConstants.number;

    driveMotor = new TalonFX(moduleConstants.driveMotorID, mapDrivetrain.CAN_BUS);
    steerMotor = new TalonFX(moduleConstants.steerMotorID, mapDrivetrain.CAN_BUS);

    absoluteEncoder = new CANCoder(moduleConstants.absoluteEncoderID, mapDrivetrain.CAN_BUS);
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

    driveCurrentLimit = new SupplyCurrentLimitConfiguration(
        prefDrivetrain.driveEnableCurrentLimit.getValue(),
        prefDrivetrain.driveHoldingCurrentLimit.getValue(),
        prefDrivetrain.drivePeakCurrentLimit.getValue(),
        prefDrivetrain.drivePeakCurrentTime.getValue());

    driveConfiguration.supplyCurrLimit = driveCurrentLimit;

    driveMotor.configAllSettings(driveConfiguration);

    driveMotor.setNeutralMode(Constants.DRIVE_NEUTRAL_MODE);
    driveMotor.setInverted(Constants.DRIVE_MOTOR_INVERT);

    // steerMotor

    steerMotor.configFactoryDefault();

    steerConfiguration.slot0.kP = prefDrivetrain.steerP.getValue();
    steerConfiguration.slot0.kI = prefDrivetrain.steerI.getValue();
    steerConfiguration.slot0.kD = prefDrivetrain.steerD.getValue();

    steerCurrentLimit = new SupplyCurrentLimitConfiguration(
        prefDrivetrain.steerEnableCurrentLimit.getValue(),
        prefDrivetrain.steerHoldingCurrentLimit.getValue(),
        prefDrivetrain.drivePeakCurrentLimit.getValue(),
        prefDrivetrain.drivePeakCurrentTime.getValue());

    steerConfiguration.supplyCurrLimit = steerCurrentLimit;

    steerMotor.configAllSettings(steerConfiguration);

    steerMotor.setNeutralMode(Constants.STEER_NEUTRAL_MODE);
    steerMotor.setInverted(Constants.STEER_MOTOR_INVERT);

    // absoluteEncoder

    absoluteEncoder.configFactoryDefault();
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
  public void setDesiredState(SwerveModuleState desiredState, boolean isDriveOpenLoop, boolean steerWhenStill) {
    SwerveModuleState state = CTREModuleState.optimize(desiredState, getState().angle);

    if (isDriveOpenLoop) {

      // calculate the percent output from the currently demanded speed and the
      // module's max speed. the currently demanded speed should never be above the
      // module's max speed given that desiredState was properly desaturated
      double percentOutput = state.speedMetersPerSecond / Constants.MAX_MODULE_SPEED;
      driveMotor.set(ControlMode.PercentOutput, percentOutput);

    } else {

      // convert the wheel speed to Falcon velocity counts
      double velocity = SN_Math.MPSToFalcon(
          state.speedMetersPerSecond,
          Constants.WHEEL_CIRCUMFERENCE,
          Constants.DRIVE_GEAR_RATIO);

      driveMotor.set(ControlMode.Velocity, velocity);
    }

    // convert angle to Falcon encoder counts
    double angle = SN_Math.degreesToFalcon(
        state.angle.getDegrees(),
        Constants.STEER_GEAR_RATIO);

    // if the module doesn't actually have any speed, don't bother steering it
    if ((Math.abs(state.speedMetersPerSecond) < (prefDrivetrain.percentToSteer.getValue() * Constants.MAX_MODULE_SPEED))
        && !steerWhenStill) {

      angle = lastAngle;
    }

    steerMotor.set(ControlMode.Position, angle);

    lastAngle = angle;
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
  public void resetSteerMotorEncodersToAbsolute() {
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

    // This could make the value negative but it doesn't matter.
    radians -= Units.degreesToRadians(absoluteEncoderOffset);

    // There are a few reasons we're subtracting here instead of just applying an
    // offset to the CANCoder.
    // Firsty, this implmentation works with any absolute encoder. You don't have to
    // rely on the encoder class on anything but getting a rotation value.
    // Secondly, and this is very related to the first point, different encoder
    // classes have identically named methods that are very different in
    // functionality. Using the bare minimum from the encoder classes and doing the
    // (very little) math ourselves minimizes opportunity for confusion

    return Rotation2d.fromRadians(radians);
  }

  /**
   * Get the raw absolute encoder position
   * 
   * @return Absolute encoder position in degrees
   */
  public double getRawAbsoluteEncoder() {
    return absoluteEncoder.getAbsolutePosition();
  }

  /**
   * Reset the drive motor encoder count to 0.
   */
  public void resetDriveEncoderCount() {
    driveMotor.setSelectedSensorPosition(0);
  }

  public double getDriveMotorOutputPercent() {
    return driveMotor.getMotorOutputPercent();
  }

  /**
   * Gets the steer motor PID goal in native units.
   * 
   * @return Steer motor PID goal
   */
  public double getSteerMotorPIDGoal() {
    return steerMotor.getClosedLoopTarget();
  }

  /**
   * Gets the steer motor PID error in native units.
   * 
   * @return Steer motor PID error
   */
  public double getSteerMotorPIDError() {
    return steerMotor.getClosedLoopError();
  }

  /**
   * Get the "last angle" of the steer motor in native motor units.
   * <p>
   * At the end of each loop, the angle that was used was set to the last angle.
   * The next loop, if the swerve module shouldn't steer, the angle gets set to
   * this last angle.
   * 
   * @return "last angle" of steer motor
   */
  public double getLastAngle() {
    return lastAngle;
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

  /**
   * Get the current position of the swerve module. Position includes a distance
   * and angle
   * 
   * @return Position of swerve module
   */
  public SwerveModulePosition getPosition() {

    double distance = SN_Math.falconToMeters(
        driveMotor.getSelectedSensorPosition(),
        Constants.WHEEL_CIRCUMFERENCE,
        Constants.DRIVE_GEAR_RATIO);

    Rotation2d angle = Rotation2d.fromDegrees(
        SN_Math.falconToDegrees(
            steerMotor.getSelectedSensorPosition(),
            Constants.STEER_GEAR_RATIO));

    return new SwerveModulePosition(distance, angle);
  }

}
