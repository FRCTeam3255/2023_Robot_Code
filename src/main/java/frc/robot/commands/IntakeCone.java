// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.RobotPreferences.prefControllers;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Intake;

public class IntakeCone extends SequentialCommandGroup {
  Collector subCollector;
  Intake subIntake;
  Arm subArm;
  SN_Blinkin leds;
  SN_F310Gamepad conOperator;

  public IntakeCone(Collector subCollector, Intake subIntake, Arm subArm, SN_F310Gamepad conOperator) {
    this.subCollector = subCollector;
    this.subIntake = subIntake;
    this.subArm = subArm;

    addCommands(
        // - Retract collector
        new InstantCommand(() -> subCollector.setPivotMotorAngle(prefCollector.pivotAngleStartingConfig.getValue())),

        // - Lower the arm so that the intake is cone level
        new InstantCommand(
            () -> subArm.setGoalAngles(prefArm.armPresetConeShoulderAngle, prefArm.armPresetConeElbowAngle)),

        // - Spin intake and rumble the controller until a game piece is collected
        Commands.parallel(
            new IntakeGamePiece(subIntake).until(subIntake::isGamePieceCollected),
            new InstantCommand(
                () -> conOperator.setRumble(RumbleType.kBothRumble, prefControllers.rumbleOutput.getValue()))
                .until(subIntake::isGamePieceCollected)),

        // - Set motors to hold speed
        new InstantCommand(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed)),

        // - Raise arm to stow position
        new InstantCommand(
            () -> subArm.setGoalAngles(prefArm.armPresetStowShoulderAngle, prefArm.armPresetStowElbowAngle)));
  }
}