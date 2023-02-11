// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.components.motors.SN_CANSparkMax;
import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.utils.SN_Math;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constCollector;
import frc.robot.RobotMap.mapCollector;
import frc.robot.RobotPreferences.prefCollector;

public class Collector extends SubsystemBase {
  SN_CANSparkMax pivotMotor;
  SN_CANSparkMax rollerMotor;

  AbsoluteEncoder pivotAbsoluteEncoder;

  TalonFXConfiguration pivotMotorConfig;

  public Collector() {
    pivotMotor = new SN_CANSparkMax(mapCollector.PIVOT_MOTOR_CAN);
    rollerMotor = new SN_CANSparkMax(mapCollector.ROLLER_MOTOR_CAN);

    pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

    pivotMotorConfig = new TalonFXConfiguration();

    configure();
  }

  public void configure() {
    pivotMotor.configFactoryDefault();
    rollerMotor.configFactoryDefault();

    pivotMotorConfig.slot0.kP = prefCollector.pivotP.getValue();
    pivotMotorConfig.slot0.kI = prefCollector.pivotI.getValue();
    pivotMotorConfig.slot0.kD = prefCollector.pivotD.getValue();

    pivotMotor.encoder.setPositionConversionFactor(SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT);

    pivotMotorConfig.forwardSoftLimitEnable = constCollector.PIVOT_FORWARD_LIMIT_ENABLE;
    pivotMotorConfig.reverseSoftLimitEnable = constCollector.PIVOT_REVERSE_LIMIT_ENABLE;

    pivotMotorConfig.forwardSoftLimitThreshold = SN_Math.degreesToFalcon(
        Units.radiansToDegrees(constCollector.PIVOT_FORWARD_LIMIT_VALUE),
        constCollector.GEAR_RATIO);
    pivotMotorConfig.reverseSoftLimitThreshold = SN_Math.degreesToFalcon(
        Units.radiansToDegrees(constCollector.PIVOT_REVERSE_LIMIT_VALUE),
        constCollector.GEAR_RATIO);

    pivotMotorConfig.slot0.allowableClosedloopError = SN_Math
        .degreesToFalcon(prefCollector.pivotTolerance.getValue(),
            constCollector.GEAR_RATIO);
    pivotMotorConfig.slot0.closedLoopPeakOutput = prefCollector.pivotMaxSpeed.getValue();

    pivotMotor.configAllSettings(pivotMotorConfig);
    resetPivotMotorToAbsolute();
  }

  /**
   * Set the angle of the collector using the pivot motor
   * 
   * @param angle Desired collector angle in degrees
   */
  public void setPivotMotorAngle(double angle) {
    double position = SN_Math.degreesToFalcon(angle, constCollector.GEAR_RATIO);
    pivotMotor.set(ControlMode.Position, position);
  }

  public Command commandSetPivotMotorAngle(SN_DoublePreference angle) {
    return Commands.runOnce(() -> setPivotMotorAngle(angle.getValue()));
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
  public double getPivotAbsoluteEncoder() {
    return pivotAbsoluteEncoder.getPosition();
  }

  /**
   * Reset the pivot motor encoder to the absolute encoder position
   */
  private void resetPivotMotorToAbsolute() {
    double absoluteEncoderCount = SN_Math.degreesToFalcon(
        getPivotAbsoluteEncoder(),
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.OUTPUT_DEBUG_VALUES) {
      SmartDashboard.putNumber("Collector Pivot Motor Encoder", pivotMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Collector Pivot Position", getPivotMotorPosition());
      SmartDashboard.putNumber("Collector Pivot Absolute Encoder", getPivotAbsoluteEncoder());
      SmartDashboard.putNumber("Collector Pivot Absolute Encoder Raw", pivotAbsoluteEncoder.getPosition());
    }
  }
}
