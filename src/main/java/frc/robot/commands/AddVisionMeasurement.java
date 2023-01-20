// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AddVisionMeasurement extends CommandBase {

  Drivetrain subDrivetrain;
  Vision subVision;

  public AddVisionMeasurement(Drivetrain subDrivetrain, Vision subVision) {

    this.subDrivetrain = subDrivetrain;
    this.subVision = subVision;
    addRequirements(subVision);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
