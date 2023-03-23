// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constArm.ArmState;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Intake;

public class IntakeCubeDeploy extends SequentialCommandGroup {

  Arm subArm;
  Collector subCollector;
  Intake subIntake;

  public IntakeCubeDeploy(Arm subArm, Collector subCollector, Intake subIntake) {

    this.subArm = subArm;
    this.subCollector = subCollector;
    this.subIntake = subIntake;

    addCommands(

        subArm.stateFromStowCommand(ArmState.COLLECTOR_MOVING),
        Commands.waitUntil(() -> subArm.isCurrentState(ArmState.COLLECTOR_MOVING)),

        new PivotCollector(subCollector, subArm, prefCollector.pivotAngleCollecting)
            .until(() -> subCollector.isAngleCollecting()),

        subArm.stateFromStowCommand(ArmState.COLLECTOR_COLLECTING_TRANSITION),
        Commands.waitUntil(() -> subArm.isCurrentState(ArmState.COLLECTOR_COLLECTING_TRANSITION)),

        Commands.runOnce(() -> subArm.setGoalState(ArmState.COLLECTOR_COLLECTING)),
        Commands.waitUntil(() -> subArm.isCurrentState(ArmState.COLLECTOR_COLLECTING)),

        new IntakeGamePiece(subIntake).until(() -> subIntake.isGamePieceCollected()));
  }
}
