// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class DriveToPosition extends CommandBase {
  Drivetrain subDrivetrain;
  Pose2d desiredPosition;
  int desiredColumn;

  public DriveToPosition(Drivetrain subDrivetrain, int desiredColumn) {
    this.subDrivetrain = subDrivetrain;
    this.desiredColumn = desiredColumn;

    addRequirements(subDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // switch (desiredColumn) {
    // case 1:
    // desiredPosition = new Pose2d(
    // new
    // Translation2d(Units.inchesToMeters(prefDrivetrain.poseGridDistanceInches.getValue()),
    // 0.5),
    // Rotation2d.fromDegrees(180));
    // break;
    // case 2:
    // desiredPosition = new Pose2d(
    // new
    // Translation2d(Units.inchesToMeters(prefDrivetrain.poseGridDistanceInches.getValue()),
    // 1.05),
    // Rotation2d.fromDegrees(180));
    // break;
    // case 3:
    // desiredPosition = new Pose2d(
    // new
    // Translation2d(Units.inchesToMeters(prefDrivetrain.poseGridDistanceInches.getValue()),
    // 1.63),
    // Rotation2d.fromDegrees(180));
    // break;
    // case 4:
    // desiredPosition = new Pose2d(
    // new
    // Translation2d(Units.inchesToMeters(prefDrivetrain.poseGridDistanceInches.getValue()),
    // 2.19),
    // Rotation2d.fromDegrees(180));
    // break;
    // case 5:
    // desiredPosition = new Pose2d(
    // new
    // Translation2d(Units.inchesToMeters(prefDrivetrain.poseGridDistanceInches.getValue()),
    // 2.7),
    // Rotation2d.fromDegrees(180));
    // break;
    // case 6:
    // desiredPosition = new Pose2d(
    // new
    // Translation2d(Units.inchesToMeters(prefDrivetrain.poseGridDistanceInches.getValue()),
    // 3.28),
    // Rotation2d.fromDegrees(180));
    // break;
    // case 7:
    // desiredPosition = new Pose2d(
    // new
    // Translation2d(Units.inchesToMeters(prefDrivetrain.poseGridDistanceInches.getValue()),
    // 3.86),
    // Rotation2d.fromDegrees(180));
    // break;
    // case 8:
    // desiredPosition = new Pose2d(
    // new
    // Translation2d(Units.inchesToMeters(prefDrivetrain.poseGridDistanceInches.getValue()),
    // 4.44),
    // Rotation2d.fromDegrees(180));
    // break;
    // case 9:
    // desiredPosition = new Pose2d(
    // new
    // Translation2d(Units.inchesToMeters(prefDrivetrain.poseGridDistanceInches.getValue()),
    // 4.97),
    // Rotation2d.fromDegrees(180));
    // break;
    // }

    // desiredPosition = new Pose2d(
    // new
    // Translation2d(Units.inchesToMeters(prefDrivetrain.poseGridDistanceInches.getValue()),
    // 1.63),
    // Rotation2d.fromDegrees(180));

    desiredPosition = new Pose2d(3.6, 5.6,
        Rotation2d.fromDegrees(0));

    SmartDashboard.putNumber("Desired position X", desiredPosition.getX());
    SmartDashboard.putNumber("Desired position Y", desiredPosition.getY());

    subDrivetrain.driveToPosition(desiredPosition);
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
