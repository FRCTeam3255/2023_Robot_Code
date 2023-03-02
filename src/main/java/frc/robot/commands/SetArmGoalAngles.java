// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmGoalAngles extends SequentialCommandGroup {
  Arm subArm;
  SN_DoublePreference desiredShoulderAngle;
  SN_DoublePreference desiredElbowAngle;

  public SetArmGoalAngles(Arm subArm, SN_DoublePreference desiredShoulderAngle, SN_DoublePreference desiredElbowAngle) {
    this.subArm = subArm;
    this.desiredShoulderAngle = desiredShoulderAngle;
    this.desiredElbowAngle = desiredElbowAngle;

    addCommands(
        Commands.sequence(
            // Stow, wait, then go
            new InstantCommand(
                () -> subArm.setGoalAngles(prefArm.armPresetStowShoulderAngle, prefArm.armPresetStowElbowAngle)),
            new WaitUntilCommand(() -> subArm.areJointsInTolerance()),
            Commands.runOnce(() -> subArm.setGoalAngles(desiredShoulderAngle, desiredElbowAngle)))

            // unless we are already going there
            .unless(() -> (subArm.getGoalShoulderAngle().getDegrees() == desiredShoulderAngle.getValue())
                && subArm.getGoalElbowAngle().getDegrees() == desiredElbowAngle.getValue()));
  }
}
