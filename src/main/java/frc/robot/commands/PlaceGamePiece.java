// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constVision.GamePiece;
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

    addCommands(
        new InstantCommand(() -> subArm.setGoalAnglesFromNumpad()),

        new InstantCommand(() -> {
          if (subIntake.getGamePieceType() == GamePiece.CUBE) {
            new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeReleaseSpeed))
                .until(() -> !subIntake.isGamePieceCollected())
                .withTimeout(prefIntake.intakeReleaseDelay.getValue());
          } else {
            // Assume game piece is a cone
            Commands.parallel(
                new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeReleaseSpeed))
                    .until(() -> !subIntake.isGamePieceCollected())
                    .withTimeout(prefIntake.intakeReleaseDelay.getValue()),

                // Move elbow to stow position to prevent hitting the node
                new InstantCommand(
                    () -> subArm.setGoalAngles(subArm.getGoalShoulderAngle(),
                        Rotation2d.fromDegrees(prefArm.armPresetStowElbowAngle.getValue()))));
          }
        }),

        // Move arm to stow position
        new InstantCommand(
            () -> subArm.setGoalAngles(prefArm.armPresetStowShoulderAngle, prefArm.armPresetStowElbowAngle)));

  }
}
