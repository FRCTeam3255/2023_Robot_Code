// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.components.motors.SN_CANSparkMax;
import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constCollector;
import frc.robot.RobotMap.mapCollector;
import frc.robot.RobotPreferences.prefCollector;

public class Collector extends SubsystemBase {
  SN_CANSparkMax pivotMotor;
  SN_CANSparkMax rollerMotor;

  Rotation2d goalPosition;

  DutyCycleEncoder pivotAbsoluteEncoder;

  TalonFXConfiguration pivotMotorConfig;

  public Collector() {
    pivotMotor = new SN_CANSparkMax(mapCollector.PIVOT_MOTOR_CAN);
    rollerMotor = new SN_CANSparkMax(mapCollector.ROLLER_MOTOR_CAN);

    goalPosition = new Rotation2d();

    pivotAbsoluteEncoder = new DutyCycleEncoder(mapCollector.PIVOT_ABSOLUTE_ENCODER_DIO);

    pivotMotorConfig = new TalonFXConfiguration();

    configure();
    Timer.delay(2.25);
    resetPivotMotorToAbsolute();
  }

  public void configure() {
    pivotMotor.configFactoryDefault();
    rollerMotor.configFactoryDefault();

    pivotMotor.encoder.setPositionConversionFactor(SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT);

    pivotMotorConfig.slot0.kP = prefCollector.pivotP.getValue();
    pivotMotorConfig.slot0.kI = prefCollector.pivotI.getValue();
    pivotMotorConfig.slot0.kD = prefCollector.pivotD.getValue();

    pivotMotorConfig.forwardSoftLimitEnable = constCollector.PIVOT_FORWARD_LIMIT_ENABLE;
    pivotMotorConfig.reverseSoftLimitEnable = constCollector.PIVOT_REVERSE_LIMIT_ENABLE;

    pivotMotorConfig.forwardSoftLimitThreshold = SN_Math.degreesToFalcon(
        Units.radiansToDegrees(constCollector.PIVOT_FORWARD_LIMIT_VALUE),
        constCollector.GEAR_RATIO);
    pivotMotorConfig.reverseSoftLimitThreshold = SN_Math.degreesToFalcon(
        Units.radiansToDegrees(constCollector.PIVOT_REVERSE_LIMIT_VALUE),
        constCollector.GEAR_RATIO);

    pivotMotorConfig.slot0.closedLoopPeakOutput = prefCollector.pivotMaxSpeed.getValue();

    pivotMotor.configAllSettings(pivotMotorConfig);

    pivotMotor.setInverted(constCollector.PIVOT_MOTOR_INVERT);
    pivotMotor.setNeutralMode(constCollector.PIVOT_MOTOR_NEUTRAL_MODE);

    rollerMotor.setInverted(constCollector.ROLLER_MOTOR_INVERT);
  }

  /**
   * Set the angle of the collector using the pivot motor
   * 
   * @param angle Desired collector angle in degrees
   */
  public void setPivotMotorAngle(double angle) {
    double position = SN_Math.degreesToFalcon(angle, constCollector.GEAR_RATIO);

    if (isPivotMotorInTolerance()) {
      pivotMotor.set(0);
    } else {
      pivotMotor.set(ControlMode.Position, position);
    }

  }

  /**
   * Set the percent output of the pivot motor
   * 
   * @param speed Percent output for pivot motor
   */
  public void setPivotMotorSpeed(double speed) {
    pivotMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Reset the pivot motor encoder counts to 0
   */
  public void resetPivotMotorEncoder() {
    pivotMotor.setSelectedSensorPosition(0);
  }

  /**
   * Get the position of the absolute encoder
   * 
   * @return Position of absolute encoder
   */
  public Rotation2d getPivotAbsoluteEncoder() {
    double rotations = pivotAbsoluteEncoder.getAbsolutePosition();
    rotations -= Units.radiansToRotations(constCollector.PIVOT_ABSOLUTE_ENCODER_OFFSET);
    rotations = MathUtil.inputModulus(rotations, -1, 1);

    if (constCollector.PIVOT_ABSOLUTE_ENCODER_INVERT) {
      return Rotation2d.fromRotations(-rotations);
    } else {
      return Rotation2d.fromRotations(rotations);
    }
  }

  /**
   * Reset the pivot motor encoder to the absolute encoder position
   */
  public void resetPivotMotorToAbsolute() {
    double absoluteEncoderCount = SN_Math.degreesToFalcon(
        getPivotAbsoluteEncoder().getDegrees(),
        constCollector.GEAR_RATIO);
    pivotMotor.setSelectedSensorPosition(absoluteEncoderCount);
  }

  /**
   * Get the position of the collector pivot joint from the motor.
   * 
   * @return Position of collector in radians
   */
  public double getPivotMotorPosition() {
    return Units.degreesToRadians(
        SN_Math.falconToDegrees(
            pivotMotor.getSelectedSensorPosition(),
            constCollector.GEAR_RATIO));
  }

  /**
   * Set the percent output of the roller motor
   * 
   * @param speed Percent output for roller motor
   */
  public void setRollerMotorSpeed(double speed) {
    rollerMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean isPivotMotorInTolerance() {
    return Math.abs(getGoalPosition().getRadians() - getPivotMotorPosition()) < (Units
        .degreesToRadians(prefCollector.pivotTolerance.getValue()));
  }

  public boolean isPivotMotorInToleranceForRoller() {
    return Math.abs(getGoalPosition().getRadians() - getPivotMotorPosition()) < (Units
        .degreesToRadians(
            prefCollector.pivotTolerance.getValue() * prefCollector.rollerToleranceMultiplier.getValue()));
  }

  /**
   * Set the goal position of the pivot motor in degrees.
   * 
   * @param goalPosition Goal position in degrees
   */
  public void setGoalPosition(SN_DoublePreference goalPosition) {
    setGoalPosition(Rotation2d.fromDegrees(goalPosition.getValue()));
  }

  /**
   * Set the goal position of the pivot motor as a Rotation2d.
   * 
   * @param goalPosition Goal position of the pivot motor
   */
  public void setGoalPosition(Rotation2d goalPosition) {
    this.goalPosition = goalPosition;
  }

  /**
   * Get the goal position of the pivot motor as a Rotation2d.
   * 
   * @return Goal position of the pivot motor
   */
  public Rotation2d getGoalPosition() {
    return goalPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.OUTPUT_DEBUG_VALUES) {
      SmartDashboard.putNumber("Collector Pivot Motor Encoder", pivotMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Collector Pivot Position", Units.radiansToDegrees(getPivotMotorPosition()));
      SmartDashboard.putNumber("Collector Pivot Absolute Encoder", getPivotAbsoluteEncoder().getDegrees());
      SmartDashboard.putNumber("Collector Pivot Absolute Encoder Raw", pivotAbsoluteEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Collector Pivot Absolute Encoder Raw Inverted",
          -pivotAbsoluteEncoder.getAbsolutePosition());
      SmartDashboard.putBoolean("Collector Pivot Is In Tolerance", isPivotMotorInTolerance());
      SmartDashboard.putNumber("Collector Goal Pivot Angle", getGoalPosition().getDegrees());

      SmartDashboard.putNumber("Collector Roller Motor Output", rollerMotor.getMotorOutputPercent());

      SmartDashboard.putNumber("Collector Pivot PID Error",
          SN_Math.falconToDegrees(pivotMotor.getClosedLoopError(), constCollector.GEAR_RATIO));
    }
  }
}
