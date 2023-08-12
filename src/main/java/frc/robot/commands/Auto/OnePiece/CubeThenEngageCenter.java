// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.OnePiece;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.commands.Engage;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeThenEngageCenter extends SequentialCommandGroup {

  Drivetrain subDrivetrain;
  Intake subIntake;

  public CubeThenEngageCenter(Drivetrain subDrivetrain, Intake subIntake) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;

    addCommands(
        Commands.runOnce(() -> subDrivetrain.resetRotation()),
        Commands.runOnce(() -> subDrivetrain.setNavXAngleAdjustment(
            subDrivetrain.scoreThenDock.getInitialHolonomicPose().getRotation().getDegrees())),

        Commands.run(() -> subIntake.setMotorSpeed(prefIntake.intakeIntakeSpeed),
            subIntake)
            .until(() -> subIntake.isGamePieceCollected()).withTimeout(5),

        Commands.waitSeconds(0.5),

        Commands.run(() -> subIntake.setMotorSpeedShoot(prefIntake.intakeReleaseSpeed.getValue()),
            subIntake)
            .withTimeout(prefIntake.intakeReleaseDelay.getValue()),

        Commands.runOnce(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed),
            subIntake),

        subDrivetrain.swerveAutoBuilder.fullAuto(subDrivetrain.scoreThenDock)
            .withTimeout(subDrivetrain.scoreThenDock.getTotalTimeSeconds()),

        new Engage(subDrivetrain));

  }
}
