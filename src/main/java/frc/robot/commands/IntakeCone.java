// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.components.SN_Blinkin;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Intake;

public class IntakeCone extends SequentialCommandGroup {
  Collector subCollector;
  Intake subIntake;
  Arm subArm;
  SN_Blinkin leds;

  public IntakeCone(Collector subCollector, Intake subIntake, Arm subArm) {
    this.subCollector = subCollector;
    this.subIntake = subIntake;
    this.subArm = subArm;

    addCommands(
        // - Lower the arm so that the intake is cone level
        Commands.runOnce(
            () -> subArm.setGoalAngles(prefArm.armPresetConeShoulderAngle, prefArm.armPresetConeElbowAngle)),

        // - Wait until the arm has reached the desired position
        Commands.waitUntil(subArm::areJointsInTolerance),

        // - Spin intake until a game piece is collected
        new IntakeGamePiece(subIntake).until(subIntake::isGamePieceCollected),

        // - Raise arm to stow position
        Commands.runOnce(
            () -> subArm.setGoalAngles(prefArm.armPresetStowShoulderAngle, prefArm.armPresetStowElbowAngle)));

  }
}