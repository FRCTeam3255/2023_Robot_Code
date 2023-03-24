// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.TwoPiece;

import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constArm.ArmState;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.commands.Engage;
import frc.robot.commands.IntakeCubeDeploy;
import frc.robot.commands.IntakeCubeRetract;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoCubeDockOpen extends SequentialCommandGroup {

  Drivetrain subDrivetrain;
  Intake subIntake;
  Arm subArm;
  Collector subCollector;

  public TwoCubeDockOpen(Drivetrain subDrivetrain, Intake subIntake, Arm subArm) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subArm = subArm;

    addCommands(
        Commands.runOnce(() -> subDrivetrain.resetRotation()),
        Commands.runOnce(() -> subDrivetrain.setNavXAngleAdjustment(
            subDrivetrain.scoreToCubeOpen.getInitialHolonomicPose().getRotation().getDegrees())),

        // Intake Current Game Piece
        Commands.run(() -> subIntake.setMotorSpeed(prefIntake.intakeIntakeSpeed), subIntake)
            .until(() -> subIntake.isGamePieceCollected()),

        // Shoot Current Game Piece
        Commands.run(() -> subArm.setGoalState(ArmState.HIGH_CUBE_SCORE_PLACE))
            .until(() -> subArm.isCurrentState(ArmState.HIGH_CUBE_SCORE_PLACE)),
        Commands.waitSeconds(0.5),

        Commands.run(() -> subIntake.setMotorSpeedShoot(prefIntake.intakeReleaseSpeed.getValue()), subIntake)
            .withTimeout(prefIntake.intakeReleaseDelay.getValue()),

        // Stow
        Commands.runOnce(() -> subArm.stowCommand()),
        Commands.runOnce(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed), subIntake),

        // Drive to collect a cube
        Commands.race(
            subDrivetrain.swerveAutoBuilder.followPath(subDrivetrain.scoreToCubeOpen)
                .withTimeout(subDrivetrain.scoreToCubeOpen.getTotalTimeSeconds()),
            new IntakeCubeDeploy(subArm, subCollector, subIntake)),

        // Drive to score
        Commands.race(
            subDrivetrain.swerveAutoBuilder.followPath(subDrivetrain.cubeToScoreOpen),
            new IntakeCubeRetract(subArm, subCollector, subIntake)),

        // Shoot Current Game Piece
        Commands.run(() -> subArm.setGoalState(ArmState.MID_CUBE_SCORE))
            .until(() -> subArm.isCurrentState(ArmState.MID_CUBE_SCORE)),
        Commands.waitSeconds(0.5),

        Commands.run(() -> subIntake.setMotorSpeedShoot(prefIntake.intakeReleaseSpeed.getValue()), subIntake)
            .withTimeout(prefIntake.intakeReleaseDelay.getValue()),

        // Stow
        Commands.runOnce(() -> subArm.stowCommand()),
        Commands.runOnce(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed), subIntake),

        // Drive to dock
        subDrivetrain.swerveAutoBuilder.followPath(subDrivetrain.scoreToDockOpen)
            .withTimeout(subDrivetrain.scoreToDockOpen.getTotalTimeSeconds()),

        // Engage
        new Engage(subDrivetrain));
  }
}
