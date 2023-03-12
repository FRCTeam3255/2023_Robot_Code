// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveToGrid extends CommandBase {
  Drivetrain subDrivetrain;
  PathPoint currentPosition;
  PPSwerveControllerCommand teleSwerveAutoController;
  PPSwerveControllerCommand onTheFlyTrajectory;

  public AutoDriveToGrid(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;
    addRequirements(subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // CURRENT POSE
    currentPosition = new PathPoint(subDrivetrain.getPose().getTranslation(), new Rotation2d(0),
        subDrivetrain.getPose().getRotation());

    // TODO: GET NUMPAD INPUT TO DECIDE YOUR PATH

    PathPlannerTrajectory cubeNode6 = PathPlanner.generatePath(
        new PathConstraints(prefDrivetrain.autoMaxAccelFeet.getValue(), prefDrivetrain.autoMaxSpeedFeet.getValue()),
        currentPosition,
        new PathPoint(new Translation2d(2, 4.42), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)));

    onTheFlyTrajectory = subDrivetrain.getOnTheFlyTrajectory(cubeNode6);

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
