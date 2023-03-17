// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.components.SN_Blinkin.PatternType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constLEDs;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class SetLEDs extends CommandBase {
  LEDs subLEDs;
  Intake subIntake;
  PatternType desiredPattern;
  Arm subArm;

  public SetLEDs(LEDs subLEDs, Intake subIntake, Arm subArm) {
    this.subLEDs = subLEDs;
    this.subIntake = subIntake;
    this.subArm = subArm;

    addRequirements(subLEDs);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (subIntake.isGamePieceCollected()) {
      desiredPattern = constLEDs.HAS_GAME_PIECE_COLOR;
    } else {
      desiredPattern = constLEDs.DEFAULT_COLOR;
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
