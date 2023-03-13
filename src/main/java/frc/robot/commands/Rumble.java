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

public class Rumble extends CommandBase {

  SN_XboxController conDriver;
  SN_XboxController conOperator;

  Intake subIntake;

  double currentTime;

  double driverRumble;
  double operatorRumble;

  boolean isGamePieceCollected;
  boolean wasGamePieceCollected;
  double timeGamePieceLeft;

  public Rumble(SN_XboxController conDriver, SN_XboxController conOperator, Intake subIntake) {
    this.conDriver = conDriver;
    this.conOperator = conOperator;

    this.subIntake = subIntake;
  }

  @Override
  public void initialize() {
    isGamePieceCollected = subIntake.isGamePieceCollected();
    wasGamePieceCollected = subIntake.isGamePieceCollected();
  }

  @Override
  public void execute() {

    driverRumble = 0;
    operatorRumble = 0;

    currentTime = Timer.getFPGATimestamp();

    isGamePieceCollected = subIntake.isGamePieceCollected();

    if (!isGamePieceCollected && wasGamePieceCollected) {
      timeGamePieceLeft = currentTime;
    }

    if (currentTime < (timeGamePieceLeft + prefControllers.rumbleDelay.getValue())) {
      driverRumble = prefControllers.rumbleOutput.getValue();
      operatorRumble = prefControllers.rumbleOutput.getValue();
    }

    wasGamePieceCollected = isGamePieceCollected;

    conDriver.setRumble(RumbleType.kBothRumble, driverRumble);
    conOperator.setRumble(RumbleType.kBothRumble, operatorRumble);
  }

  @Override
  public void end(boolean interrupted) {
    conDriver.setRumble(RumbleType.kBothRumble, 0);
    conOperator.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
