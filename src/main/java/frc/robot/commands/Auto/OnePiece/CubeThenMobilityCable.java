// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.OnePiece;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constArm.ArmState;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeThenMobilityCable extends SequentialCommandGroup {

  Drivetrain subDrivetrain;
  Intake subIntake;
  Arm subArm;

  public CubeThenMobilityCable(Drivetrain subDrivetrain, Intake subIntake, Arm subArm) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subArm = subArm;

    addCommands(
        Commands.runOnce(() -> subDrivetrain.resetRotation()),
        Commands.runOnce(() -> subDrivetrain.setNavXAngleAdjustment(
            subDrivetrain.scoreToCubeCable.getInitialHolonomicPose().getRotation().getDegrees())),

        Commands.run(() -> subIntake.setMotorSpeed(prefIntake.intakeIntakeSpeed), subIntake)
            .until(() -> subIntake.isGamePieceCollected()).withTimeout(5),

        Commands
            .run(() -> subArm.setGoalState(ArmState.MID_STOWED))
            .until(() -> subArm.isCurrentState(ArmState.MID_STOWED)),

        Commands.run(() -> subArm.setGoalState(ArmState.HIGH_CUBE_SCORE_PLACE))
            .until(() -> subArm.isCurrentState(ArmState.HIGH_CUBE_SCORE_PLACE)),
        Commands.waitSeconds(0.5),

        Commands.run(() -> subIntake.setMotorSpeedShoot(prefIntake.intakeReleaseSpeed.getValue()), subIntake)
            .withTimeout(prefIntake.intakeReleaseDelay.getValue()),

        Commands.runOnce(() -> subArm.setGoalState(ArmState.HIGH_STOWED)),

        Commands.runOnce(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed), subIntake),

        subDrivetrain.swerveAutoBuilder.fullAuto(subDrivetrain.scoreToCubeCable));

  }
}
