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
        new InstantCommand(() -> subArm.setGoalAnglesFromNumpad()),
        new WaitUntilCommand(() -> subArm.areJointsInTolerance()),

        // Lower the arm UNLESS
        // Its a hybrid node, we provided no scoring level, or its a cube node
        new InstantCommand(() -> subArm.setGoalAngles(
            Rotation2d.fromDegrees(
                subArm.getGoalShoulderAngle().getDegrees() - prefArm.armShoulderLoweringAngle.getValue()),
            Rotation2d.fromDegrees(
                subArm.getGoalElbowAngle().getDegrees() - prefArm.armElbowLoweringAngle.getValue())))
            .unless(() -> subArm.scoringLevel == ScoringLevel.HYBRID || subArm.scoringLevel == ScoringLevel.NONE
                || subArm.isCubeNode()),
        new WaitUntilCommand(() -> subArm.areJointsInTolerance())
            .unless(() -> subArm.scoringLevel == ScoringLevel.HYBRID || subArm.scoringLevel == ScoringLevel.NONE
                || subArm.isCubeNode()),

        new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeReleaseSpeed), subIntake)
            .until(() -> !subIntake.isGamePieceCollected()),
        new WaitCommand(prefIntake.intakeReleaseDelay.getValue()),
        new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed), subIntake));
  }
}
