// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constArm.ArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class IntakeFloor extends SequentialCommandGroup {

  Arm subArm;
  Intake subIntake;

  public IntakeFloor(Arm subArm, Intake subIntake) {
    this.subArm = subArm;
    this.subIntake = subIntake;

    addCommands(
        Commands.runOnce(() -> subArm.setGoalState(ArmState.FLOOR_INTAKE)),
        Commands.waitUntil(() -> subArm.getCurrentState() == ArmState.FLOOR_INTAKE),
        new IntakeGamePiece(subIntake).until(() -> subIntake.isGamePieceCollected()),
        Commands.runOnce(() -> subArm.setGoalState(ArmState.STOWED)));
  }
}
