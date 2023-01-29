// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_F310Gamepad;

import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;
import com.frcteam3255.joystick.SN_SwitchboardStick;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.constControllers;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefChargerTreads;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Drive;
import frc.robot.subsystems.ChargerTreads;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {

  private final ChargerTreads subChargerTreads = new ChargerTreads();
  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Intake subIntake = new Intake();
  private final Arm subArm = new Arm();
  private final Vision subVision = new Vision();
  private final Collector subCollector = new Collector();

  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER_USB);
  private final SN_F310Gamepad conOperator = new SN_F310Gamepad(mapControllers.OPERATOR_USB);
  private final SN_SwitchboardStick conSwitchboard = new SN_SwitchboardStick(mapControllers.SWITCHBOARD_USB);
  private final SN_Blinkin leds = new SN_Blinkin(mapControllers.BLINKIN_PWM);

  public RobotContainer() {

    subDrivetrain.setDefaultCommand(new Drive(subDrivetrain, conDriver));
    subVision.setDefaultCommand(new AddVisionMeasurement(subDrivetrain, subVision));
    subCollector.setDefaultCommand(
        new RunCommand(
            () -> subCollector.setPivotMotorSpeed(
                MathUtil.applyDeadband(
                    conOperator.getAxisRSY(),
                    constControllers.OPERATOR_RIGHT_STICK_Y_DEADBAND)),
            subCollector));

    configureBindings();
  }

  // Leds

  // While held, Leds will change to given color, and turn off on release
  private void configureBindings() {

    // Driver

    // "reset gyro" for field relative but actually resets the orientation at a
    // higher level
    conDriver.btn_A
        .onTrue(Commands.runOnce(
            () -> subDrivetrain.resetPose(new Pose2d(subDrivetrain.getPose().getTranslation(), new Rotation2d(0)))));
    conDriver.btn_B
        .onTrue(Commands.runOnce(
            () -> subDrivetrain.resetPose(new Pose2d())));

    // Operator

    // Set the arm to a preset position (example bind, may not be necessary for comp
    // bindings)
    conOperator.btn_A
        .whileTrue(Commands.runOnce(() -> subArm.setJointPositions(prefArm.shoulderPreset, prefArm.elbowPreset), subArm)
            .repeatedly())
        .onFalse((Commands.runOnce(() -> subArm.setShoulderPercentOutput(0), subArm)));

    // Switchboard

    // Sets LED color to "violet" to indicate a purple game piece (cube) is being
    // requested
    conSwitchboard.btn_1
        .onTrue(Commands.runOnce(() -> leds.setPattern(PatternType.Violet)))
        .onFalse(Commands.runOnce(() -> leds.setPattern(PatternType.Black)));

    // Sets LED color to "yellow" to indicate a yellow game piece (cone) is being
    // requested
    conSwitchboard.btn_2
        .onTrue(Commands.runOnce(() -> leds.setPattern(PatternType.Yellow)))
        .onFalse(Commands.runOnce(() -> leds.setPattern(PatternType.Black)));

    // Test keybinds

    // Spin the Collector roller motor while held
    conOperator.btn_B
        .onTrue(Commands.runOnce(() -> subCollector.setRollerMotorSpeed(prefCollector.rollerSpeed.getValue())))
        .onFalse(Commands.runOnce(() -> subCollector.setRollerMotorSpeed(0)));

    // Set Collector to starting config
    conOperator.btn_X
        .onTrue(
            Commands.runOnce(
                () -> subCollector.setPivotMotorAngle(prefCollector.pivotAngleStartingConfig.getValue())));

    // Set Collector Rollers to intake height
    conOperator.btn_Y
        .onTrue(
            Commands
                .runOnce(() -> subCollector.setPivotMotorAngle(prefCollector.pivotAngleCubeCollecting.getValue())));

    // Set Collector Rollers to climbing position
    conOperator.btn_A
        .onTrue(
            Commands.runOnce(() -> subCollector.setPivotMotorAngle(prefCollector.pivotAngleClimb.getValue())));

    // Spin Charger treads
    conOperator.btn_RBump
        .onTrue(Commands.runOnce(() -> subChargerTreads.setMotorSpeed(prefChargerTreads.motorSpeed.getValue())))
        .onFalse(Commands.runOnce(() -> subChargerTreads.setMotorSpeed(0)));

    // Rotate drivetrain wheels in charge station orientation
    conOperator.POV_North
        .onTrue(Commands.runOnce(
            () -> subDrivetrain.drive(new Pose2d(0, 0, new Rotation2d(prefDrivetrain.chargeRotation.getValue())))))
        .onFalse(Commands.runOnce(() -> subDrivetrain.drive(new Pose2d(0, 0, new Rotation2d(0)))));

    // Spin drivetrain wheels to go onto the charge station
    conOperator.btn_RBump
        .onTrue(Commands.runOnce(() -> subDrivetrain.drive(new Pose2d(prefDrivetrain.chargeVelocityX.getValue(),
            prefDrivetrain.chargeVelocityY.getValue(), new Rotation2d(prefDrivetrain.chargeRotation.getValue())))))
        .onFalse(Commands.runOnce(() -> subDrivetrain.drive(new Pose2d(0, 0, new Rotation2d(0)))));
  }

  public Command getAutonomousCommand() {
    PathPlannerTrajectory testPath = PathPlanner.loadPath("testPath",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));

    return subDrivetrain.swerveAutoBuilder.fullAuto(testPath)
        .andThen(new InstantCommand(() -> subDrivetrain.neutralDriveOutputs(), subDrivetrain));
  }
}
