// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_F310Gamepad;
import com.frcteam3255.joystick.SN_XboxController;
import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.utils.SN_Debug;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constVision.GamePiece;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.RobotPreferences.prefControllers;
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
  SN_XboxController conOperator;

  SN_DoublePreference shoulderDegrees;
  SN_DoublePreference elbowDegrees;

  public PlaceGamePiece(Arm subArm, Collector subCollector, Intake subIntake, SN_DoublePreference shoulderDegrees,
      SN_DoublePreference elbowDegrees, SN_XboxController conOperator) {

    this.subArm = subArm;
    this.subCollector = subCollector;
    this.subIntake = subIntake;
    this.conOperator = conOperator;

    this.shoulderDegrees = shoulderDegrees;
    this.elbowDegrees = elbowDegrees;

    addCommands(
        // Retract collector
        // new InstantCommand(() ->
        // subCollector.setPivotMotorAngle(prefCollector.pivotAngleStartingConfig.getValue())),

        // Move arm to desired position
        new InstantCommand(() -> subArm.setGoalAngles(this.shoulderDegrees, this.elbowDegrees)),

        // Detect game piece type
        new InstantCommand(() -> {
          if (subIntake.getGamePieceType() == GamePiece.CONE) {
            Commands.parallel(
                // If game piece is a cone, then release, rumble contoller, and back away in
                // parallel
                new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeReleaseSpeed))
                    .until(() -> !subIntake.isGamePieceCollected())
                    .withTimeout(prefIntake.intakeReleaseDelay.getValue())
                    .andThen(new InstantCommand(
                        () -> conOperator.setRumble(RumbleType.kBothRumble, prefControllers.rumbleOutput.getValue()))),

                // Move elbow to stow position to prevent hitting the node
                new InstantCommand(
                    () -> subArm.setGoalAngles(this.shoulderDegrees, prefArm.armPresetStowElbowAngle)));

          } else if (subIntake.getGamePieceType() == GamePiece.CUBE) {
            // If game piece is a cube, then angle elbow downward, release, and rumble
            // controller
            new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeReleaseSpeed))
                .until(() -> !subIntake.isGamePieceCollected())
                .withTimeout(prefIntake.intakeReleaseDelay.getValue())
                .andThen(new InstantCommand(
                    () -> conOperator.setRumble(RumbleType.kBothRumble, prefControllers.rumbleOutput.getValue())));
          }
        }),

        // Move arm to stow position and stop rumbling
        Commands.parallel(
            new InstantCommand(
                () -> subArm.setGoalAngles(prefArm.armPresetStowShoulderAngle, prefArm.armPresetStowElbowAngle)),
            new InstantCommand(() -> conOperator.setRumble(RumbleType.kBothRumble, 0))));
  }
}
