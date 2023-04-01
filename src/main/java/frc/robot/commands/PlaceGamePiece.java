// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constArm.ArmState;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class PlaceGamePiece extends SequentialCommandGroup {

  Arm subArm;
  Intake subIntake;

  public PlaceGamePiece(Arm subArm, Intake subIntake) {
    this.subArm = subArm;
    this.subIntake = subIntake;

    addCommands(
        Commands.runOnce(() -> subArm.setGoalState(ArmState.HIGH_CONE_SCORE_LOWERED))
            .unless(() -> !subArm.isCurrentState(ArmState.HIGH_CONE_SCORE)),

        Commands.waitUntil(() -> subArm.isCurrentState(ArmState.HIGH_CONE_SCORE_LOWERED))
            .unless(() -> !subArm.isGoalState(ArmState.HIGH_CONE_SCORE_LOWERED)),

        Commands.runOnce(() -> subArm.setGoalState(ArmState.MID_CONE_SCORE_LOWERED))
            .unless(() -> !subArm.isCurrentState(ArmState.MID_CONE_SCORE)),

        Commands.waitUntil(() -> subArm.isCurrentState(ArmState.MID_CONE_SCORE_LOWERED))
            .unless(() -> !subArm.isGoalState(ArmState.MID_CONE_SCORE_LOWERED)),

        subIntake.releaseCommand());

  }
}
