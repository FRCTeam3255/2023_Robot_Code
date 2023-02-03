// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Intake;

public class IntakeGamePiece extends CommandBase {
  Intake subIntake;
  SN_DoublePreference speed;

  public IntakeGamePiece(Intake subIntake) {
    this.subIntake = subIntake;

    addRequirements(this.subIntake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    speed = prefIntake.intakeIntakeSpeed;

    if (subIntake.isGamePieceCollected()) {
      speed = prefIntake.intakeHoldSpeed;
    }

    subIntake.setMotorSpeed(speed);

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
