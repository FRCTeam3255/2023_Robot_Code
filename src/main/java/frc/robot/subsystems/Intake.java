// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.frcteam3255.components.motors.SN_CANSparkMax;
import com.frcteam3255.joystick.SN_XboxController;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;
import frc.robot.RobotPreferences.prefControllers;
import frc.robot.RobotPreferences.prefIntake;

public class Intake extends SubsystemBase {

  SN_CANSparkMax leftMotor;
  SN_CANSparkMax rightMotor;
  DigitalInput limitSwitch;

  Rumble subRumble;

  Boolean hasGamePiece;

  public Intake(Rumble subRumble) {
    leftMotor = new SN_CANSparkMax(mapIntake.INTAKE_LEFT_MOTOR_CAN);
    rightMotor = new SN_CANSparkMax(mapIntake.INTAKE_RIGHT_MOTOR_CAN);

    limitSwitch = new DigitalInput(mapIntake.INTAKE_LIMIT_SWITCH_DIO);

    this.subRumble = subRumble;

    hasGamePiece = false;

    configure();
  }

  public void configure() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setInverted(constIntake.LEFT_MOTOR_INVERTED);
    rightMotor.setInverted(constIntake.RIGHT_MOTOR_INVERTED);

    leftMotor.setNeutralMode(constIntake.NEUTRAL_MODE);
    rightMotor.setNeutralMode(constIntake.NEUTRAL_MODE);
  }

  public boolean getLimitSwitch() {
    return constIntake.LIMIT_SWITCH_INVERTED ? !limitSwitch.get() : limitSwitch.get();
  }

  public boolean isGamePieceCollected() {
    return getLimitSwitch();
  }

  public void setMotorSpeed(SN_DoublePreference speed) {
    setMotorSpeed(speed.getValue());
  }

  public void setMotorSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setMotorSpeedShoot(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed * prefIntake.intakeLeftMotorMultiplier.getValue());
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  public Command releaseCommand() {
    return this.run(() -> setMotorSpeed(prefIntake.intakeReleaseSpeed));
  }

  public Command holdCommand() {
    return this.run(() -> setMotorSpeed(prefIntake.intakeHoldSpeed));
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Is Game Piece Collected", isGamePieceCollected());

    if (Constants.OUTPUT_DEBUG_VALUES) {
      SmartDashboard.putBoolean("Intake Limit Switch", getLimitSwitch());
    }

    if (!hasGamePiece && getLimitSwitch()) {
      hasGamePiece = true;
    }
    if (hasGamePiece && !getLimitSwitch()) {
      hasGamePiece = false;
      subRumble.setInstantRumble();
    }
  }
}
