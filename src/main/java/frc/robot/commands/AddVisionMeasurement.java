// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AddVisionMeasurement extends CommandBase {

  Drivetrain subDrivetrain;
  Vision subVision;
  Pose2d estimatedPose;
  double latency;

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
    Optional<Pair<Pose3d, Double>> result = subVision.getPoseFromVision(subDrivetrain.getPose());

    if (result.isPresent()) {
      estimatedPose = result.get().getFirst().toPose2d();
      latency = result.get().getSecond();
      subDrivetrain.addVisionMeasurement(estimatedPose, Timer.getFPGATimestamp() - latency);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
