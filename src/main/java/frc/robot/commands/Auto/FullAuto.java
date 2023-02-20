// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.commands.Dock;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.MoveArm;
import frc.robot.commands.PlaceGamePiece;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullAuto extends SequentialCommandGroup {

  Arm arm;
  Collector collector;
  Drivetrain drivetrain;
  Intake intake;

  PlaceGamePiece placeGamePiece;
  IntakeCone intakeCone;
  MoveArm moveArm;
  Dock dock;

  public FullAuto(Arm subArm, Collector subCollector, Drivetrain subDrivetrain, Intake subIntake,
      IntakeCone intakeCone, MoveArm moveArm) {

    arm = subArm;
    collector = subCollector;
    drivetrain = subDrivetrain;
    intake = subIntake;

    placeGamePiece = new PlaceGamePiece(subArm, subCollector, subIntake, null, null);
    intakeCone = new IntakeCone(subCollector, subIntake, subArm);

    addCommands(
        placeGamePiece,

        Commands.parallel(
            // TODO: drivetrain.twoConePath should be the updated FullAuto path
            drivetrain.swerveAutoBuilder.fullAuto(drivetrain.twoConePath)
                .andThen(new InstantCommand(() -> drivetrain.neutralDriveOutputs(), drivetrain)),

            // Delay to make sure intake doesn't collide after placing cone
            intakeCone.withTimeout(2)),

        dock);
  }
}
