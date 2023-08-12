// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapElevator;
import frc.robot.RobotPreferences.prefElevator;

public class Elevator extends SubsystemBase {

  TalonFX leftMotor;
  TalonFX rightMotor;

  TalonFXConfiguration config;

  public Elevator() {
    leftMotor = new TalonFX(mapElevator.LEFT_MOTOR_CAN);
    rightMotor = new TalonFX(mapElevator.RIGHT_MOTOR_CAN);

    configure();
  }

  public void configure() {

    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.configAllSettings(config);
    rightMotor.configAllSettings(config);

    config.slot0.kF = prefElevator.elevatorF.getValue();
    config.slot0.kP = prefElevator.elevatorP.getValue();
    config.slot0.kI = prefElevator.elevatorI.getValue();
    config.slot0.kD = prefElevator.elevatorD.getValue();

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    leftMotor.follow(rightMotor);
  }

  public void setElevatorSpeed(double speed) {
    // don't go past the max position
    if (getElevatorDistanceFeet() > prefElevator.elevatorMaxPos.getValue() && speed > 0) {
      speed = 0;
    }

    // don't go past the min position
    if (getElevatorDistanceFeet() < prefElevator.elevatorMinPos.getValue() && speed > 0) {
      speed = 0;
    }

    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setElevatorPosition(double position) {
    position = MathUtil.clamp(position, prefElevator.elevatorMinPos.getValue(),
        prefElevator.elevatorMaxPos.getValue());

    leftMotor.set(ControlMode.Position, position);
    rightMotor.set(ControlMode.Position, position);
  }

  public double getElevatorEncoderCounts() {
    return rightMotor.getSelectedSensorPosition();
  }

  public double getElevatorDistanceFeet() {
    return getElevatorEncoderCounts() / prefElevator.elevatorEncoderCountsPerFoot.getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Encoder Counts", getElevatorEncoderCounts());
    SmartDashboard.putNumber("Elevator Distance Feet", getElevatorDistanceFeet());
  }
}
