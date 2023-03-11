// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.constControllers.ScoringLevel;
import frc.robot.RobotPreferences.prefArm;
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
        new InstantCommand(() -> subArm.setGoalStateFromNumpad()),
        new WaitUntilCommand(() -> subArm.areJointsInTolerance()),

        new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeReleaseSpeed), subIntake)
            .until(() -> !subIntake.isGamePieceCollected()),
        new WaitCommand(prefIntake.intakeReleaseDelay.getValue()),
        new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed), subIntake));
  }
}
