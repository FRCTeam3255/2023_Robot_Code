// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {

  ColorSensorV3 intakeColorSensor;
  ColorMatch colorMatcher;
  Color coneColor = new Color(0.34509, 0.51764, 0.13333);
  Color cubeColor = new Color(0.22745, 0.39607, 0.37254);

  public Intake() {
    intakeColorSensor = new ColorSensorV3(mapIntake.COLOR_SENSOR_I2C);
    colorMatcher = new ColorMatch();

    colorMatcher.setConfidenceThreshold(0.95);
    // CONE
    colorMatcher.addColorMatch(coneColor);
    // CUBE
    colorMatcher.addColorMatch(cubeColor);
  }

  public Constants.gamePiece hasGamePiece() {
    Constants.gamePiece currentGamePiece = Constants.gamePiece.NONE;
    Color detectedColor = intakeColorSensor.getColor();
    ColorMatchResult currentColor = colorMatcher.matchColor(detectedColor);

    if (currentColor == null) {
      return currentGamePiece;
    } else if (currentColor.color == coneColor) {
      currentGamePiece = Constants.gamePiece.CONE;
    } else if (currentColor.color == cubeColor) {
      currentGamePiece = Constants.gamePiece.CUBE;
    }

    return currentGamePiece;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Color Sensor Color", intakeColorSensor.getColor().toHexString());
    SmartDashboard.putNumber("Color Sensor Red", intakeColorSensor.getRed());
    SmartDashboard.putNumber("Color Sensor Green", intakeColorSensor.getGreen());
    SmartDashboard.putNumber("Color Sensor Blue", intakeColorSensor.getBlue());
    SmartDashboard.putNumber("Color Sensor Proximity", intakeColorSensor.getProximity());
    SmartDashboard.putString("Current Game Piece", hasGamePiece().toString());
  }
}
