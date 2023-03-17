// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefControllers;
import frc.robot.subsystems.Intake;

public class SetRumble extends CommandBase {
  SN_XboxController conDriver;
  SN_XboxController conOperator;

  Intake subIntake;

  Boolean hadGamePiece;

  Double rumbleOutput;

  Double timeGamePieceLeft;

  public SetRumble(SN_XboxController conDriver, SN_XboxController conOperator, Intake subIntake) {
    this.conDriver = conDriver;
    this.conOperator = conOperator;

    this.subIntake = subIntake;

    hadGamePiece = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rumbleOutput = 0.0;
    timeGamePieceLeft = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!hadGamePiece && subIntake.getLimitSwitch()) {
      hadGamePiece = true;
    } else if (hadGamePiece && !subIntake.getLimitSwitch()) {
      hadGamePiece = false;
      timeGamePieceLeft = Timer.getFPGATimestamp();
      rumbleOutput = prefControllers.rumbleOutput.getValue();
    }

    if (Timer.getFPGATimestamp() > timeGamePieceLeft + prefControllers.rumbleDelay.getValue()) {
      rumbleOutput = 0.0;
    }

    conDriver.setRumble(RumbleType.kBothRumble, rumbleOutput);
    conOperator.setRumble(RumbleType.kBothRumble, rumbleOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
