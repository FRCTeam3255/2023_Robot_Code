// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constControllers.ScoringLevel;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.RobotPreferences.prefIntake;
// import frc.robot.commands.IntakeCone;
// import frc.robot.commands.PlaceGamePiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeConeDock extends SequentialCommandGroup {

  Drivetrain subDrivetrain;
  Intake subIntake;
  Arm subArm;

  public CubeConeDock(Drivetrain subDrivetrain, Intake subIntake, Arm subArm) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subArm = subArm;

    // subArm.scoringLevel = ScoringLevel.HIGH;

    addCommands(
        // Prep to shoot cube
        Commands.waitSeconds(1),
        subDrivetrain.swerveAutoBuilder.resetPose(subDrivetrain.cubeThenDock),
        // Commands
        // .run(() -> subArm.setGoalAngles(prefArm.armShootCubeHighShoulderAngle,
        // prefArm.armShootCubeHighElbowAngle))
        // .until(() -> subArm.areJointsInTolerance()),
        Commands.waitSeconds(0.5),

        // Shoot cube
        Commands.run(() -> subIntake.setMotorSpeed(prefIntake.intakeShootSpeedHigh), subIntake)
            .until(() -> !subIntake.isGamePieceCollected()),
        Commands.waitSeconds(prefIntake.intakeReleaseDelay.getValue()),
        Commands.runOnce(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed), subIntake),

        // Drive to cone
        subDrivetrain.swerveAutoBuilder.fullAuto(subDrivetrain.cubeThenStagingMark)
            .andThen(Commands.runOnce(() -> subDrivetrain.setDefenseMode(), subDrivetrain)),

        // Intake cone
        // new IntakeCone(subIntake, subArm).until(() ->
        // subIntake.isGamePieceCollected()),
        Commands.runOnce(() -> subIntake.setMotorSpeed(prefIntake.intakeHoldSpeed), subIntake),

        // Drive to go prep
        subDrivetrain.swerveAutoBuilder.fullAuto(subDrivetrain.stagingMarkThenPrep)
            .andThen(Commands.runOnce(() -> subDrivetrain.setDefenseMode(), subDrivetrain)),

        // Prep
        // Commands
        // .run(
        // () -> subArm.setGoalAngles(prefArm.armPresetConeHighShoulderAngle,
        // prefArm.armPresetConeHighElbowAngle))
        // .until(() -> subArm.areJointsInTolerance()),
        Commands.waitSeconds(0.5),

        // Drive to go Place
        subDrivetrain.swerveAutoBuilder.fullAuto(subDrivetrain.prepThenBottomCone)
            .andThen(Commands.runOnce(() -> subDrivetrain.setDefenseMode(), subDrivetrain)),

        // Place
        // new PlaceGamePiece(subArm, subIntake),

        // TODO: DRIVE BACK AND STOW

        // Go dock
        subDrivetrain.swerveAutoBuilder.fullAuto(subDrivetrain.bottomConeThenDock)
            .andThen(Commands.runOnce(() -> subDrivetrain.setDefenseMode(), subDrivetrain))

    );
  }
}
