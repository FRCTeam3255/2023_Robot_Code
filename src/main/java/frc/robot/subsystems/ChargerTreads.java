// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.frcteam3255.components.motors.SN_CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constChargerTreads;
import frc.robot.RobotMap.mapChargerTreads;
import frc.robot.RobotPreferences.prefChargerTreads;

public class ChargerTreads extends SubsystemBase {

  SN_CANSparkMax leftMotor;
  SN_CANSparkMax rightMotor;

  public ChargerTreads() {
    leftMotor = new SN_CANSparkMax(mapChargerTreads.TREADS_LEFT_MOTOR_CAN);
    rightMotor = new SN_CANSparkMax(mapChargerTreads.TREADS_RIGHT_MOTOR_CAN);

    configure();
  }

  public void configure() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setInverted(constChargerTreads.LEFT_MOTOR_INVERTED);
    rightMotor.setInverted(constChargerTreads.RIGHT_MOTOR_INVERTED);

    leftMotor.setNeutralMode(constChargerTreads.NEUTRAL_MODE);
    rightMotor.setNeutralMode(constChargerTreads.NEUTRAL_MODE);
  }

  public void runMotors() {
    leftMotor.set(ControlMode.PercentOutput, prefChargerTreads.chargerSpeed.getValue());
    rightMotor.set(ControlMode.PercentOutput, prefChargerTreads.chargerSpeed.getValue());
  }

  public void stopMotors() {
    leftMotor.neutralOutput();
    rightMotor.neutralOutput();
  }

  @Override
  public void periodic() {
  }
}
