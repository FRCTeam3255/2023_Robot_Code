// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.components.motors.SN_TalonFX;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constCollector;
import frc.robot.RobotMap.mapCollector;
import frc.robot.RobotPreferences.prefCollector;

public class Collector extends SubsystemBase {
  SN_TalonFX pivotMotor;
  SN_TalonFX intakeMotor;

  TalonFXConfiguration config;

  public Collector() {
    pivotMotor = new SN_TalonFX(mapCollector.PIVOT_MOTOR_CAN);
    intakeMotor = new SN_TalonFX(mapCollector.INTAKE_MOTOR_CAN);

    config = new TalonFXConfiguration();
    configure();
  }

  public void configure() {
    pivotMotor.configFactoryDefault();
    intakeMotor.configFactoryDefault();

    config.slot0.kP = prefCollector.collectorP.getValue();
    config.slot0.kI = prefCollector.collectorI.getValue();
    config.slot0.kD = prefCollector.collectorD.getValue();

    config.slot0.allowableClosedloopError = SN_Math
        .degreesToFalcon(prefCollector.collectorAllowableClosedLoopErrorDegrees.getValue(), constCollector.GEAR_RATIO);
    config.slot0.closedLoopPeakOutput = prefCollector.collectorClosedLoopPeakOutput.getValue();

    pivotMotor.configAllSettings(config);
  }

  public void spinIntakeMotor(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  // TODO: Set PID values
  public void setPivotMotorPosition(double position) {
    pivotMotor.set(ControlMode.Position, position);
  }

  public void setPivotMotorSpeed(double speed) {
    pivotMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("EncoderCounts", pivotMotor.getSelectedSensorPosition());
  }
}
