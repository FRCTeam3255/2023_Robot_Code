// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.constControllers.ScoringColumn;
import frc.robot.Constants.constControllers.ScoringLevel;
import frc.robot.Constants.constVision.GamePiece;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class PrepPlacement extends InstantCommand {
  Arm subArm;
  Drivetrain subDrivetrain;
  Intake subIntake;
  GamePiece currentGamePiece;
  ScoringColumn scoringColumn;
  ScoringLevel scoringLevel;

  boolean isCubeColumn;
  SN_DoublePreference shoulderDegrees;
  SN_DoublePreference elbowDegrees;

  public PrepPlacement(Arm subArm, Drivetrain subDrivetrain, Intake subIntake,
      ScoringColumn scoringColumn, ScoringLevel scoringLevel) {
    this.subArm = subArm;
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.scoringColumn = scoringColumn;
    this.scoringLevel = scoringLevel;
  }

  @Override
  public void initialize() {
    currentGamePiece = subIntake.getGamePieceType();
    isCubeColumn = (scoringColumn == ScoringColumn.SECOND || scoringColumn == ScoringColumn.FIFTH
        || scoringColumn == ScoringColumn.EIGHTH);

    if (isCubeColumn && currentGamePiece != GamePiece.CONE) {
      // Node is good, do driving code
    } else {
      if (scoringLevel == ScoringLevel.HYBRID) {
        // Node is actually good sike
      } else {
        // Node is bad, do something else ig
        // TODO: Decide what to do if the node is invalid
      }
    }

    switch (scoringLevel) {
      case HYBRID:
        shoulderDegrees = prefArm.armPresetLowElbowAngle;
        elbowDegrees = prefArm.armPresetLowShoulderAngle;
        break;
      case MID:
        shoulderDegrees = prefArm.armPresetMidElbowAngle;
        elbowDegrees = prefArm.armPresetMidShoulderAngle;
        break;
      case HIGH:
        shoulderDegrees = prefArm.armPresetHighElbowAngle;
        elbowDegrees = prefArm.armPresetHighShoulderAngle;
        break;
      case NONE:
        // TODO: WHAT DO?!??!??!?
    }
    // If position is valid, drive
    // TODO: Add driving code

    // and move the arm
    subArm.setGoalAngles(shoulderDegrees, elbowDegrees);
  }
}
