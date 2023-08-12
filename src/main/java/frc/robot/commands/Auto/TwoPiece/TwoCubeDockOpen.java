// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.TwoPiece;

import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.commands.Engage;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoCubeDockOpen extends SequentialCommandGroup {

  Drivetrain subDrivetrain;
  Intake subIntake;
  Collector subCollector;

  public TwoCubeDockOpen(Drivetrain subDrivetrain, Intake subIntake) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;

    addCommands(
        Commands.runOnce(() -> subDrivetrain.resetRotation()),
        Commands.runOnce(() -> subDrivetrain.setNavXAngleAdjustment(
            subDrivetrain.scoreToCubeOpen.getInitialHolonomicPose().getRotation().getDegrees())),

        // Intake Current Game Piece
        Commands.run(() -> subIntake.setMotorSpeed(prefIntake.intakeIntakeSpeed), subIntake)
            .until(() -> subIntake.isGamePieceCollected()),

        // Shoot Current Game Piece

        Commands.waitSeconds(0.5),

        Commands.run(() -> subIntake.setMotorSpeedShoot(prefIntake.intakeReleaseSpeed.getValue()), subIntake)
            .withTimeout(prefIntake.intakeReleaseDelay.getValue()),

        // Stow
        Commands.runOnce(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed), subIntake),

        // Drive to collect a cube
        Commands.race(
            subDrivetrain.swerveAutoBuilder.fullAuto(subDrivetrain.scoreToCubeOpen)
                .withTimeout(subDrivetrain.scoreToCubeOpen.getTotalTimeSeconds())),

        // Drive to score
        Commands.race(
            subDrivetrain.swerveAutoBuilder.fullAuto(subDrivetrain.cubeToDockOutsideOpen)
                .withTimeout(subDrivetrain.cubeToDockOutsideOpen.getTotalTimeSeconds())),

        // Shoot Current Game Piece

        Commands.waitSeconds(0.5),

        Commands.run(() -> subIntake.setMotorSpeedShoot(prefIntake.intakeShootSpeedChargeStation.getValue()), subIntake)
            .withTimeout(prefIntake.intakeReleaseDelay.getValue()),

        // Stow
        Commands.runOnce(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed), subIntake),

        // Engage
        new Engage(subDrivetrain));
  }
}
