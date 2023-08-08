// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.frcteam3255.components.motors.SN_CANSparkMax;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;
import frc.robot.RobotPreferences.prefIntake;

public class Intake extends SubsystemBase {

  SN_CANSparkMax leftMotor;
  SN_CANSparkMax rightMotor;
  DigitalInput limitSwitch;

  private ShuffleboardTab intakeTab = Shuffleboard.getTab("SuperShuffle");
  private GenericEntry shuffleGamePieceCollected = intakeTab
      .add("Piece Collected", false)
      .withWidget("Boolean Box")
      .withSize(1, 3)
      .withPosition(7, 0)
      .withProperties(Map.of("colorWhenTrue", "#4d74ff", "colorWhenFalse", "#000000"))
      .getEntry();

  public Intake() {

    leftMotor = new SN_CANSparkMax(mapIntake.INTAKE_LEFT_MOTOR_CAN);
    rightMotor = new SN_CANSparkMax(mapIntake.INTAKE_RIGHT_MOTOR_CAN);

    limitSwitch = new DigitalInput(mapIntake.INTAKE_LIMIT_SWITCH_DIO);

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

    shuffleGamePieceCollected.setBoolean(isGamePieceCollected());

    SmartDashboard.putBoolean("Intake Is Game Piece Collected", isGamePieceCollected());

    if (Constants.OUTPUT_DEBUG_VALUES) {
      SmartDashboard.putBoolean("Intake Limit Switch", getLimitSwitch());
    }
  }
}
