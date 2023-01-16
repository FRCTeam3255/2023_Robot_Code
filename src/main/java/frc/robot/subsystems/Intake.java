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
import frc.robot.Constants.GamePiece;
import frc.robot.RobotMap.mapIntake;
import frc.robot.RobotPreferences.prefIntake;

public class Intake extends SubsystemBase {

  ColorSensorV3 intakeColorSensor;
  ColorMatch colorMatcher;
  Color coneColor;
  Color cubeColor;

  public Intake() {
    intakeColorSensor = new ColorSensorV3(mapIntake.COLOR_SENSOR_I2C);
    colorMatcher = new ColorMatch();

    configure();
  }

  public void configure() {
    coneColor = new Color(Constants.coneColorR, Constants.coneColorG, Constants.coneColorB);
    cubeColor = new Color(Constants.cubeColorR, Constants.cubeColorG, Constants.cubeColorB);

    colorMatcher.setConfidenceThreshold(prefIntake.colorMatcherConfidence.getValue());
    colorMatcher.addColorMatch(coneColor);
    colorMatcher.addColorMatch(cubeColor);
  }

  public GamePiece hasGamePiece() {
    GamePiece currentGamePiece = GamePiece.NONE;
    Color detectedColor = intakeColorSensor.getColor();
    ColorMatchResult currentColor = colorMatcher.matchColor(detectedColor);

    if (currentColor == null) {
      return currentGamePiece;
    } else if (currentColor.color == coneColor) {
      currentGamePiece = GamePiece.CONE;
    } else if (currentColor.color == cubeColor) {
      currentGamePiece = GamePiece.CUBE;
    }

    return currentGamePiece;
  }

  @Override
  public void periodic() {
    if (Constants.OUTPUT_DEBUG_VALUES) {
      SmartDashboard.putString("Color Sensor Color", intakeColorSensor.getColor().toHexString());
      SmartDashboard.putNumber("Color Sensor Red", intakeColorSensor.getRed());
      SmartDashboard.putNumber("Color Sensor Green", intakeColorSensor.getGreen());
      SmartDashboard.putNumber("Color Sensor Blue", intakeColorSensor.getBlue());
      SmartDashboard.putNumber("Color Sensor Proximity", intakeColorSensor.getProximity());
    }
    SmartDashboard.putString("Current Game Piece", hasGamePiece().toString());
  }
}
