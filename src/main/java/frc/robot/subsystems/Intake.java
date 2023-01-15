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
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {

  ColorSensorV3 intakeColorSensor;
  ColorMatch colorMatcher;

  // I am 90% sure this is not the right place to do this, maybe constants
  public enum gamePiece {
    NONE, CUBE, CONE
  }

  public Intake() {
    intakeColorSensor = new ColorSensorV3(mapIntake.COLOR_SENSOR_I2C);
    colorMatcher = new ColorMatch();

    // TODO: MAKE THIS NOT BAD
    colorMatcher.setConfidenceThreshold(0.3);

    // TODO: Change these to more exact colors

    // CONE
    colorMatcher.addColorMatch(Color.kYellow);
    // CUBE
    colorMatcher.addColorMatch(Color.kViolet);
  }

  public gamePiece hasGamePiece() {
    gamePiece currentGamePiece = gamePiece.NONE;
    Color detectedColor = intakeColorSensor.getColor();
    ColorMatchResult currentColor = colorMatcher.matchColor(detectedColor);

    if (currentColor == null) {
      return currentGamePiece;
    } else if (currentColor.color == Color.kYellow) {
      currentGamePiece = gamePiece.CONE;
    } else if (currentColor.color == Color.kViolet) {
      currentGamePiece = gamePiece.CUBE;
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
