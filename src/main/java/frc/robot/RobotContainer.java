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
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.constControllers;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefChargerTreads;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.commands.SetLEDs;
import frc.robot.commands.MoveArm;
import frc.robot.commands.intakeCube;
import frc.robot.subsystems.ChargerTreads;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefIntake;
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
  private final LEDs subLEDs = new LEDs();

  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER_USB);
  private final SN_F310Gamepad conOperator = new SN_F310Gamepad(mapControllers.OPERATOR_USB);
  private final SN_SwitchboardStick conSwitchboard = new SN_SwitchboardStick(mapControllers.SWITCHBOARD_USB);

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
    subIntake.setDefaultCommand(subIntake.holdCommand());
    subLEDs.setDefaultCommand(new SetLEDs(subLEDs, subIntake));
    subArm.setDefaultCommand(new MoveArm(subArm, subCollector));

    configureBindings();
  }

  public void configureNeutralModes() {
    subArm.setJointsNeutralMode();
  }
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

    // while true do robot oriented, default to field oriented
    conDriver.btn_LBump
        .whileTrue(Commands.runOnce(() -> subDrivetrain.setRobotRelative()))
        .onFalse(Commands.runOnce(() -> subDrivetrain.setFieldRelative()));

    // Operator

    // Run IntakeCube command
    conOperator.btn_LBump.onTrue(new intakeCube(subArm, subCollector, subIntake, leds));

    // TODO: Run IntakeCone command (btn_RB)
    // TODO: Run PrepPlace command (btn_LT)
    // TODO: Run PlaceGamePiece command (btn_RT)

    // Set stow Arm preset
    conOperator.btn_B.onTrue(Commands
        .runOnce(() -> subArm.setGoalAngles(prefArm.armPresetStowShoulderAngle, prefArm.armPresetStowElbowAngle)));

    // Set low Arm preset
    conOperator.btn_A.onTrue(Commands
        .runOnce(() -> subArm.setGoalAngles(prefArm.armPresetLowShoulderAngle, prefArm.armPresetLowElbowAngle)));

    // Set mid Arm preset
    conOperator.btn_X.onTrue(Commands
        .runOnce(() -> subArm.setGoalAngles(prefArm.armPresetMidShoulderAngle, prefArm.armPresetMidElbowAngle)));

    // Set high Arm preset
    conOperator.btn_Y.onTrue(Commands
        .runOnce(() -> subArm.setGoalAngles(prefArm.armPresetHighShoulderAngle, prefArm.armPresetHighElbowAngle)));

    // TODO: Create button to manually adjust arm
    // shoulder: btn_LS
    // elbow: btn_RS

    // Set Collector to starting config and stop the rollers
    conOperator.POV_North
        .onTrue(
            Commands.runOnce(
                () -> subCollector.setPivotMotorAngle(prefCollector.pivotAngleStartingConfig.getValue()))
                .alongWith(Commands.runOnce(() -> subCollector.setRollerMotorSpeed(0))));

    // Set Collector rollers to intake height and spin the rollers
    conOperator.POV_South
        .onTrue(
            Commands
                .runOnce(() -> subCollector.setPivotMotorAngle(prefCollector.pivotAngleCubeCollecting.getValue()))
                .alongWith(
                    Commands.runOnce(() -> subCollector.setRollerMotorSpeed(prefCollector.rollerSpeed.getValue()))));

    // Spin the Intake forward
    conOperator.btn_Start.onTrue(Commands.runOnce(() -> subIntake.setMotorSpeed(prefIntake.intakeIntakeSpeed)));

    // Spin the Intake in reverse
    conOperator.btn_Back.onTrue(Commands.runOnce(() -> subIntake.setMotorSpeed(prefIntake.intakeReleaseSpeed)));
  }

  public Command getAutonomousCommand() {
    return subDrivetrain.swerveAutoBuilder.fullAuto(subDrivetrain.twoConePath)
        .andThen(new InstantCommand(() -> subDrivetrain.neutralDriveOutputs(),
            subDrivetrain));
  }
}
