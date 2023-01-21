// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.frcteam3255.components.motors.SN_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapCollector;
import frc.robot.RobotPreferences.prefCollector;

public class Collector extends SubsystemBase {
  SN_TalonFX pivotMotor;
  SN_TalonFX intakeMotor;

  public Collector() {
    pivotMotor = new SN_TalonFX(mapCollector.PIVOT_MOTOR_CAN);
    intakeMotor = new SN_TalonFX(mapCollector.INTAKE_MOTOR_CAN);

    configure();
  }

  public void configure() {
    pivotMotor.configFactoryDefault();
    intakeMotor.configFactoryDefault();
  }

  public void spinIntakeMotor(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  // TODO: Set PID values
  public void setPivotMotorPosition(double position) {
    pivotMotor.set(ControlMode.Position, position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("EncoderCounts", pivotMotor.getSelectedSensorPosition());
  }
}
