// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.constControllers.ScoringLevel;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceGamePiece extends SequentialCommandGroup {

  Arm subArm;
  Collector subCollector;
  Intake subIntake;

  public PlaceGamePiece(Arm subArm, Collector subCollector, Intake subIntake) {

    this.subArm = subArm;
    this.subCollector = subCollector;
    this.subIntake = subIntake;

    // TODO: Replace ALL ARM WAIT COMMANDS to arm is within tolerance when that
    // method is pushed to QA

    addCommands(
        new InstantCommand(() -> subArm.setGoalAnglesFromNumpad()),
        new WaitCommand(1),
        // if (subIntake.getGamePieceType() == GamePiece.CUBE) {
        // new InstantCommand(() ->
        // subIntake.setMotorSpeed(prefIntake.intakeReleaseSpeed))
        // .until(() -> !subIntake.isGamePieceCollected())
        // .withTimeout(prefIntake.intakeReleaseDelay.getValue());
        // } else {

        // Assume game piece is a cone (Hopefully it just works with a cube)

        // Lower the arm slightly if its not a hybrid node or we never gave a level
        new InstantCommand(() -> subArm.setGoalAngles(
            Rotation2d.fromDegrees(
                subArm.getGoalShoulderAngle().getDegrees() - prefArm.armShoulderLoweringAngle.getValue()),
            Rotation2d.fromDegrees(
                subArm.getGoalElbowAngle().getDegrees() - prefArm.armElbowLoweringAngle.getValue())))
            .unless(() -> subArm.scoringLevel == ScoringLevel.HYBRID || subArm.scoringLevel == ScoringLevel.NONE),
        new WaitCommand(2)
            .unless(() -> subArm.scoringLevel == ScoringLevel.HYBRID || subArm.scoringLevel == ScoringLevel.NONE),

        new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeReleaseSpeed), subIntake)
            .until(() -> !subIntake.isGamePieceCollected()),
        new WaitCommand(prefIntake.intakeReleaseDelay.getValue()),
        new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed), subIntake));
  }
}
