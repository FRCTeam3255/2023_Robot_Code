// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class IntakeTippedCone extends SequentialCommandGroup {
  Collector subCollector;
  Drivetrain subDrivetrain;
  Intake subIntake;
  Arm subArm;

  public IntakeTippedCone(Collector subCollector, Drivetrain subdDrivetrain, Intake subIntake, Arm subArm) {
    this.subCollector = subCollector;
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subArm = subArm;

    addCommands(
        // Retract collector
        new InstantCommand(() -> subCollector.setPivotMotorAngle(prefCollector.pivotAngleStartingConfig.getValue())),

        // Lower the arm so that the intake is cone level
        new InstantCommand(
            () -> subArm.setGoalAngles(prefArm.armPresetConeShoulderAngle, prefArm.armPresetConeElbowAngle)),

        // Spin intake until a game piece is collected
        new IntakeGamePiece(subIntake).until(subIntake::isGamePieceCollected),

        // Move backwards until game piece is collected
        // TODO: Add driving code in parallel with intake

        // Set motors to hold speed
        new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed)),

        // Raise arm to stow position
        new InstantCommand(
            () -> subArm.setGoalAngles(prefArm.armPresetStowShoulderAngle, prefArm.armPresetStowElbowAngle)));
  }
}
