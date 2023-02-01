// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.frcteam3255.components.motors.SN_CANSparkMax;
import com.frcteam3255.preferences.SN_DoublePreference;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constIntake;
import frc.robot.Constants.GamePiece;
import frc.robot.RobotMap.mapIntake;
import frc.robot.RobotPreferences.prefIntake;

public class Intake extends SubsystemBase {

  SN_CANSparkMax leftMotor;
  SN_CANSparkMax rightMotor;
  DigitalInput limitSwitch;
  ColorSensorV3 colorSensor;
  ColorMatch colorMatcher;
  Color coneColor;
  Color cubeColor;

  public Intake() {
    leftMotor = new SN_CANSparkMax(mapIntake.INTAKE_LEFT_MOTOR_CAN);
    rightMotor = new SN_CANSparkMax(mapIntake.INTAKE_RIGHT_MOTOR_CAN);

    colorSensor = new ColorSensorV3(mapIntake.COLOR_SENSOR_I2C);
    colorMatcher = new ColorMatch();

    limitSwitch = new DigitalInput(mapIntake.LIMIT_SWITCH_DIO);

    coneColor = new Color(constIntake.coneColorR, constIntake.coneColorG, constIntake.coneColorB);
    cubeColor = new Color(constIntake.cubeColorR, constIntake.cubeColorG, constIntake.cubeColorB);

    colorMatcher.addColorMatch(coneColor);
    colorMatcher.addColorMatch(cubeColor);

    configure();
  }

  public void configure() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setInverted(constIntake.LEFT_MOTOR_INVERTED);
    rightMotor.setInverted(constIntake.RIGHT_MOTOR_INVERTED);

    colorMatcher.setConfidenceThreshold(prefIntake.colorMatcherConfidence.getValue());
  }

  public GamePiece getGamePieceType() {
    ColorMatchResult currentColor = colorMatcher.matchColor(colorSensor.getColor());

    if (currentColor == null) {
      return GamePiece.NONE;
    } else if (currentColor.color == coneColor) {
      return GamePiece.CONE;
    } else if (currentColor.color == cubeColor) {
      return GamePiece.CUBE;
    }

    return GamePiece.HUH;
  }

  public boolean getLimitSwitch() {
    return constIntake.LIMIT_SWITCH_INVERT ? !limitSwitch.get() : limitSwitch.get();
  }

  public boolean isGamePieceCollected() {
    if (getLimitSwitch()) {
      return true;
    }

    else if (colorSensor.getProximity() <= prefIntake.gamePieceProximity.getValue()) {
      return true;
    }

    else if (getGamePieceType() == GamePiece.CONE || getGamePieceType() == GamePiece.CUBE) {
      return true;
    }
    return false;
  }

  public void setMotorSpeed(SN_DoublePreference speed) {
    setMotorSpeed(speed.getValue());
  }

  public void setMotorSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
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

    SmartDashboard.putString("Current Game Piece", getGamePieceType().toString());

    if (Constants.OUTPUT_DEBUG_VALUES) {
      SmartDashboard.putString("Intake Color Sensor Color", colorSensor.getColor().toHexString());
      SmartDashboard.putNumber("Intake Color Sensor Red", colorSensor.getRed());
      SmartDashboard.putNumber("Intake Color Sensor Green", colorSensor.getGreen());
      SmartDashboard.putNumber("Intake Color Sensor Blue", colorSensor.getBlue());
      SmartDashboard.putNumber("Intake Color Sensor Proximity", colorSensor.getProximity());
      SmartDashboard.putBoolean("Intake Limit Switch", getLimitSwitch());
      SmartDashboard.putBoolean("Intake Is Game Piece Collected", isGamePieceCollected());
    }
  }
}
