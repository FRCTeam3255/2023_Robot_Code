// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.frcteam3255.components.motors.SN_CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constCharger;
import frc.robot.RobotMap.mapCharger;
import frc.robot.RobotPreferences.prefCharger;

public class Charger extends SubsystemBase {

  SN_CANSparkMax leftMotor;
  SN_CANSparkMax rightMotor;

  public Charger() {
    leftMotor = new SN_CANSparkMax(mapCharger.LEFT_MOTOR_CAN);
    rightMotor = new SN_CANSparkMax(mapCharger.RIGHT_MOTOR_CAN);

    configure();
  }

  public void configure() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setInverted(constCharger.LEFT_MOTOR_INVERTED);
    rightMotor.setInverted(constCharger.RIGHT_MOTOR_INVERTED);

    leftMotor.setNeutralMode(constCharger.NEUTRAL_MODE);
    rightMotor.setNeutralMode(constCharger.NEUTRAL_MODE);
  }

  public void runMotors() {
    leftMotor.set(ControlMode.PercentOutput, prefCharger.chargerSpeed.getValue());
    rightMotor.set(ControlMode.PercentOutput, prefCharger.chargerSpeed.getValue());
  }

  public void stopMotors() {
    leftMotor.neutralOutput();
    rightMotor.neutralOutput();
  }

  @Override
  public void periodic() {
  }
}
