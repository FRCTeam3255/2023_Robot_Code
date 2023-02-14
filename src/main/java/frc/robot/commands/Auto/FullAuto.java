// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.commands.MoveArm;
import frc.robot.commands.intakeCube;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullAuto extends SequentialCommandGroup {

  Arm arm;
  Collector collector;
  Drivetrain drivetrain;
  Intake intake;

  intakeCube intakeCube;
  MoveArm moveArm;

  // TODO:
  // Place cube
  // Drive/Rotate
  // Intake Cube ✓
  // Drive to charge station
  // Dock/Engage

  public FullAuto(Arm subArm, Collector subCollector, Drivetrain subDrivetrain, Intake subIntake,
      intakeCube intakeCube, MoveArm moveArm) {

    arm = subArm;
    collector = subCollector;
    drivetrain = subDrivetrain;
    intake = subIntake;

    intakeCube = new intakeCube(subArm, subCollector, subIntake, null);
    moveArm = new MoveArm(subArm, subCollector);

    addCommands(
        // Placeholder
        Commands.parallel(moveArm),

        Commands.parallel(intakeCube));
  }
}
