// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;
import com.frcteam3255.joystick.SN_SwitchboardStick;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.constControllers.ScoringColumn;
import frc.robot.Constants.constControllers.ScoringLevel;
import frc.robot.Constants.constVision.GamePiece;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.SetLEDs;
import frc.robot.commands.MoveArm;
import frc.robot.commands.PivotCollector;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);
  private final SN_SwitchboardStick conSwitchboard = new SN_SwitchboardStick(mapControllers.SWITCHBOARD_USB);
  private final SN_SwitchboardStick conNumpad = new SN_SwitchboardStick(mapControllers.NUMPAD_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Arm subArm = new Arm();
  private final Intake subIntake = new Intake();
  private final Collector subCollector = new Collector();
  private final Vision subVision = new Vision();
  private final LEDs subLEDs = new LEDs();

  public RobotContainer() {

    subDrivetrain
        .setDefaultCommand(new Drive(
            subDrivetrain,
            conDriver.axis_LeftY,
            conDriver.axis_LeftX,
            conDriver.axis_RightX,
            conDriver.axis_RightTrigger,
            conDriver.btn_Y,
            conDriver.btn_B,
            conDriver.btn_A,
            conDriver.btn_X));
    subArm.setDefaultCommand(new MoveArm(subArm, subCollector, conOperator.axis_LeftY, conOperator.axis_RightY));
    subIntake.setDefaultCommand(subIntake.holdCommand());
    subCollector.setDefaultCommand(new PivotCollector(subCollector));
    subVision.setDefaultCommand(new AddVisionMeasurement(subDrivetrain, subVision));
    subLEDs.setDefaultCommand(new SetLEDs(subLEDs, subIntake, subArm.desiredGamePiece));

    configureBindings();

    Timer.delay(2.5);
    resetToAbsolutePositions();
  }

  public void configureNeutralModes() {
    subArm.setJointsNeutralMode();
  }

  /**
   * Reset all the applicable motor encoders to their corresponding absolute
   * encoder.
   */
  public void resetToAbsolutePositions() {
    subDrivetrain.resetSteerMotorEncodersToAbsolute();
    subArm.resetJointEncodersToAbsolute();
    subCollector.resetPivotMotorToAbsolute();
  }

  private void configureBindings() {

    // Driver

    // "reset gyro" for field relative but actually resets the orientation at a
    // higher level
    conDriver.btn_Back
        .onTrue(Commands.runOnce(
            () -> subDrivetrain.resetRotation()));

    // while true do robot oriented, default to field oriented
    conDriver.btn_LeftBumper
        .whileTrue(Commands.runOnce(() -> subDrivetrain.setRobotRelative()))
        .onFalse(Commands.runOnce(() -> subDrivetrain.setFieldRelative()));

    // Set defense mode
    conDriver.btn_RightBumper.whileTrue(Commands.run(() -> subDrivetrain.setDefenseMode(), subDrivetrain));

    // Operator

    // Run IntakeCube command
    conOperator.btn_LeftBumper.whileTrue(new IntakeCube(subArm, subIntake, subCollector));

    // Run IntakeCone command
    conOperator.btn_RightBumper.whileTrue(new IntakeCone(subCollector, subIntake, subArm));

    // Prep Place
    conOperator.btn_LeftTrigger.whileTrue(Commands.run(() -> subArm.setGoalAnglesFromNumpad()).repeatedly());

    // Place Game piece
    conOperator.btn_RightTrigger.whileTrue(new PlaceGamePiece(subArm, subCollector, subIntake));

    // Set stow Arm preset
    conOperator.btn_B.onTrue(Commands
        .runOnce(() -> subArm.setGoalAngles(prefArm.armPresetStowShoulderAngle, prefArm.armPresetStowElbowAngle)));

    // Set low Arm preset
    conOperator.btn_A.onTrue(Commands
        .runOnce(
            () -> subArm.setGoalAngles(prefArm.armPresetLowShoulderAngle, prefArm.armPresetLowElbowAngle)));

    // Set Shelf Arm preset
    conOperator.btn_Y.onTrue(Commands
        .runOnce(() -> subArm.setGoalAngles(prefArm.armPresetShoulderShelf, prefArm.armPresetElbowShelf)));

    // TODO: Create button to manually adjust arm
    // shoulder: btn_LS
    // elbow: btn_RS

    // Set Collector to starting config and stop the rollers
    conOperator.btn_North
        .onTrue(Commands.runOnce(() -> subCollector.setGoalPosition(prefCollector.pivotAngleStartingConfig)));

    // Set Collector rollers to intake height and spin the rollers
    conOperator.btn_South
        .onTrue(Commands.runOnce(() -> subCollector.setGoalPosition(prefCollector.pivotAngleCubeCollecting)));

    // Set the LEDs to "We want a cone"
    conOperator.btn_West.onTrue(Commands.runOnce(() -> subArm.desiredGamePiece = GamePiece.CONE));

    // Set the LEDs to "We want a cube"
    conOperator.btn_East.onTrue(Commands.runOnce(() -> subArm.desiredGamePiece = GamePiece.CUBE));

    // Spin the Intake forward
    conOperator.btn_Start
        .whileTrue(Commands.run(() -> subIntake.setMotorSpeed(prefIntake.intakeIntakeSpeed), subIntake));

    // Spin the Intake in reverse
    conOperator.btn_Back
        .whileTrue(Commands.run(() -> subIntake.setMotorSpeed(prefIntake.intakeReleaseSpeed), subIntake));

    // Numpad
    conNumpad.btn_1.onTrue(Commands.runOnce(() -> subArm.scoringColumn = ScoringColumn.FIRST));
    conNumpad.btn_2.onTrue(Commands.runOnce(() -> subArm.scoringColumn = ScoringColumn.SECOND));
    conNumpad.btn_3.onTrue(Commands.runOnce(() -> subArm.scoringColumn = ScoringColumn.THIRD));
    conNumpad.btn_4.onTrue(Commands.runOnce(() -> subArm.scoringColumn = ScoringColumn.FOURTH));
    conNumpad.btn_5.onTrue(Commands.runOnce(() -> subArm.scoringColumn = ScoringColumn.FIFTH));
    conNumpad.btn_6.onTrue(Commands.runOnce(() -> subArm.scoringColumn = ScoringColumn.SIXTH));
    conNumpad.btn_7.onTrue(Commands.runOnce(() -> subArm.scoringColumn = ScoringColumn.SEVENTH));
    conNumpad.btn_8.onTrue(Commands.runOnce(() -> subArm.scoringColumn = ScoringColumn.EIGHTH));
    conNumpad.btn_9.onTrue(Commands.runOnce(() -> subArm.scoringColumn = ScoringColumn.NINTH));

    conNumpad.btn_10.onTrue(Commands.runOnce(() -> subArm.scoringLevel = ScoringLevel.HYBRID));
    conNumpad.btn_11.onTrue(Commands.runOnce(() -> subArm.scoringLevel = ScoringLevel.MID));
    conNumpad.btn_12.onTrue(Commands.runOnce(() -> subArm.scoringLevel = ScoringLevel.HIGH));
  }

  public Command getAutonomousCommand() {
    return subDrivetrain.swerveAutoBuilder.fullAuto(subDrivetrain.twoConePath)
        .andThen(new InstantCommand(() -> subDrivetrain.neutralDriveOutputs(),
            subDrivetrain));
  }
}
