// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.components.SN_Blinkin.PatternType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constLEDs;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class SetLEDs extends CommandBase {
  LEDs subLEDs;
  Intake subIntake;
  PatternType desiredPattern;

  public SetLEDs(LEDs subLEDs, Intake subIntake) {
    this.subLEDs = subLEDs;
    this.subIntake = subIntake;

    addRequirements(subLEDs);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // TODO: MAKE THINGS PREFERENCES

    switch (subIntake.getGamePieceType()) {
      case CONE:
        desiredPattern = constLEDs.hasConeColor;
        break;
      case CUBE:
        desiredPattern = constLEDs.hasCubeColor;
        break;
      case HUH:
        desiredPattern = constLEDs.failureColor;
        break;
      case NONE:
        if (subIntake.isGamePieceCollected()) {
          // We have a game piece but the color sensor failed
          // TODO: create fallthrough logic
          break;
        }
        if (true) {
          // We don't have a game and we WANT something
          // TODO: Create a conditional that returns CONE or CUBE
        }
        // We dont have a game piece or want a game piece; default color
        desiredPattern = constLEDs.defaultColor;
        break;
    }

    subLEDs.setLEDPattern(desiredPattern);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
