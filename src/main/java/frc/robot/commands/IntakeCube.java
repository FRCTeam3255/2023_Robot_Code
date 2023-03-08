// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.RobotPreferences.prefArm;
// import frc.robot.RobotPreferences.prefCollector;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Collector;
// import frc.robot.subsystems.Intake;

// public class IntakeCube extends SequentialCommandGroup {

// Arm subArm;
// Intake subIntake;
// Collector subCollector;

// public IntakeCube(Arm subArm, Intake subIntake, Collector subCollector) {

// addCommands(
// // arm stow
// Commands.runOnce(() -> subArm.setGoalAngles(
// prefArm.armPresetStowShoulderAngle,
// prefArm.armPresetStowElbowAngle)),
// Commands.waitUntil(() -> subArm.areJointsInTolerance()),

// // collector deploy
// Commands.runOnce(() ->
// subCollector.setGoalPosition(prefCollector.pivotAngleCubeCollecting)),
// Commands.waitUntil(() -> subCollector.isPivotMotorInTolerance()),

// // arm out
// Commands.runOnce(
// () -> subArm.setGoalAngles(
// prefArm.armPresetStraightShoulderAngle,
// prefArm.armPresetStraightElbowAngle)),
// Commands.waitUntil(() -> subArm.areJointsInTolerance()),

// // arm collect cube, until cube collected
// Commands.runOnce(
// () -> subArm.setGoalAngles(
// prefArm.armPresetCollectorShoulderAngle,
// prefArm.armPresetCollectorElbowAngle)),
// Commands.waitUntil(() -> subArm.areJointsInTolerance()),

// new IntakeGamePiece(subIntake).until(() -> subIntake.isGamePieceCollected()),

// // raise shoulder, keep elbow still
// Commands.runOnce(
// () -> subArm.setGoalAngles(
// prefArm.armPresetPostCollectorShoulderAngle,
// prefArm.armPresetCollectorElbowAngle)),
// Commands.waitUntil(() -> subArm.areJointsInTolerance()),

// // stick elbow out while keeping shoulder still
// Commands.runOnce(
// () -> subArm.setGoalAngles(
// prefArm.armPresetPostCollectorShoulderAngle,
// prefArm.armPresetStraightElbowAngle)),
// Commands.waitUntil(() -> subArm.areJointsInTolerance()),

// // collector retract
// Commands.runOnce(() ->
// subCollector.setGoalPosition(prefCollector.pivotAngleStartingConfig)),
// Commands.waitUntil(() -> subCollector.isPivotMotorInTolerance()),

// // arm stow
// Commands.runOnce(() -> subArm.setGoalAngles(
// prefArm.armPresetStowShoulderAngle,
// prefArm.armPresetStowElbowAngle)),
// Commands.waitUntil(() -> subArm.areJointsInTolerance()));
// }
// }
