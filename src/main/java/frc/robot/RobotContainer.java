// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RobotContainer {

  private final Intake subIntake = new Intake();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
