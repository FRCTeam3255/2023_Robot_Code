// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.components.motors.SN_TalonFX;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constCollector;
import frc.robot.RobotMap.mapCollector;
import frc.robot.RobotPreferences.prefCollector;

public class Collector extends SubsystemBase {
  SN_TalonFX pivotMotor;
  SN_TalonFX rollerMotor;

  DutyCycleEncoder pivotEncoder;

  TalonFXConfiguration config;

  public Collector() {
    pivotMotor = new SN_TalonFX(mapCollector.PIVOT_MOTOR_CAN);
    rollerMotor = new SN_TalonFX(mapCollector.ROLLER_MOTOR_CAN);

    pivotEncoder = new DutyCycleEncoder(mapCollector.PIVOT_ABSOLUTE_ENCODER_DIO);

    config = new TalonFXConfiguration();
    configure();
  }

  public void configure() {
    pivotMotor.configFactoryDefault();
    rollerMotor.configFactoryDefault();

    config.slot0.kP = prefCollector.collectorP.getValue();
    config.slot0.kI = prefCollector.collectorI.getValue();
    config.slot0.kD = prefCollector.collectorD.getValue();

    config.slot0.allowableClosedloopError = SN_Math
        .degreesToFalcon(prefCollector.collectorAllowableClosedLoopErrorDegrees.getValue(), constCollector.GEAR_RATIO);
    config.slot0.closedLoopPeakOutput = prefCollector.collectorClosedLoopPeakOutput.getValue();

    pivotMotor.configAllSettings(config);
    resetCollectortoAbsolute();
  }

  public void spinRollerMotor(double speed) {
    rollerMotor.set(ControlMode.PercentOutput, speed);
  }

  // TODO: Set PID values
  public void setPivotMotorAngle(double angle) {
    angle = SN_Math.degreesToFalcon(angle, constCollector.GEAR_RATIO);
    setPivotMotorPosition(angle);
  }

  public void setPivotMotorPosition(double position) {
    pivotMotor.set(ControlMode.Position, position);
  }

  public double getCollectorAbsoluteEncoder() {
    return pivotEncoder.getAbsolutePosition();
  }

  private void resetCollectortoAbsolute() {
    double absoluteEncoderCount = SN_Math.degreesToFalcon(getCollectorAbsoluteEncoder(),
        constCollector.GEAR_RATIO);
    pivotMotor.setSelectedSensorPosition(absoluteEncoderCount);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Collector Pivot Absolute Encoder", getCollectorAbsoluteEncoder());
    SmartDashboard.putNumber("Collector Pivot Motor Encoder", pivotMotor.getSelectedSensorPosition());
  }
}
