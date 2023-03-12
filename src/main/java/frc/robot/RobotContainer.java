// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;
import com.frcteam3255.joystick.SN_SwitchboardStick;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constArm.ArmState;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeFloor;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.commands.MoveArm;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.SetLEDs;
import frc.robot.commands.Auto.OnePiece.CenterCube;
import frc.robot.commands.Auto.OnePiece.CubeThenDock;
import frc.robot.commands.Auto.OnePiece.CubeThenMobilityCable;
import frc.robot.commands.Auto.OnePiece.CubeThenMobilityOpen;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);
  private final SN_SwitchboardStick conSwitchboard = new SN_SwitchboardStick(mapControllers.SWITCHBOARD_USB);
  private final SN_SwitchboardStick conNumpad = new SN_SwitchboardStick(mapControllers.NUMPAD_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Arm subArm = new Arm();
  private final Intake subIntake = new Intake();
  // private final Collector subCollector = new Collector();
  private final Vision subVision = new Vision();
  private final LEDs subLEDs = new LEDs();

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  private static DigitalInput pracBotSwitch = new DigitalInput(9);

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_X_DEADBAND);

    subDrivetrain
        .setDefaultCommand(new Drive(
            subDrivetrain,
            subArm,
            conDriver.axis_LeftY,
            conDriver.axis_LeftX,
            conDriver.axis_RightX,
            conDriver.axis_RightTrigger,
            conDriver.btn_Y,
            conDriver.btn_B,
            conDriver.btn_A,
            conDriver.btn_X));
    subArm.setDefaultCommand(new MoveArm(subArm, conOperator.axis_LeftY, conOperator.axis_RightY));
    subIntake.setDefaultCommand(subIntake.holdCommand());
    // subCollector.setDefaultCommand(new PivotCollector(subCollector));
    subVision.setDefaultCommand(new AddVisionMeasurement(subDrivetrain,
        subVision));
    subLEDs.setDefaultCommand(new SetLEDs(subLEDs, subIntake, subArm));

    configureBindings();
    configureAutoSelector();

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
    // subCollector.resetPivotMotorToAbsolute();
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

    conDriver.btn_RightBumper.whileTrue(Commands.run(() -> subDrivetrain.setDefenseMode(), subDrivetrain));

    // Operator

    // Run IntakeCube command
    // conOperator.btn_LeftBumper.whileTrue(new IntakeCube(subArm, subIntake,
    // subCollector));

    // Intake Floor (rbump)
    conOperator.btn_RightBumper.whileTrue(new IntakeFloor(subArm, subIntake));

    // Set stow Arm preset (b)
    conOperator.btn_B.onTrue(Commands.runOnce(() -> subArm.setGoalState(ArmState.STOWED)));

    // Set low Arm preset (a)
    conOperator.btn_A.onTrue(Commands.runOnce(() -> subArm.setGoalState(ArmState.HYBRID_SCORE)))
        .onTrue(Commands.runOnce(() -> subArm.setDesiredNode(7)));

    // Set Shelf Arm preset (y)
    conOperator.btn_Y.onTrue(Commands.runOnce(() -> subArm.setGoalState(ArmState.SHELF_INTAKE)))
        .whileTrue(new IntakeGamePiece(subIntake));

    // prep place (x)
    conOperator.btn_X.onTrue(Commands.runOnce(() -> subArm.setStateFromDesiredNode()));

    // Place Game piece (rt)
    conOperator.btn_RightTrigger.whileTrue(new PlaceGamePiece(subArm, subIntake));

    // Spin the Intake forward
    conOperator.btn_Start
        .whileTrue(new IntakeGamePiece(subIntake));

    // Spin the Intake in reverse (back)
    conOperator.btn_Back
        .whileTrue(subIntake.releaseCommand());

    conOperator.btn_North.whileTrue(Commands.runOnce(() -> subArm.configure()));

    // numpad

    // mid cone
    conNumpad.btn_9.onTrue(Commands.runOnce(() -> {
      subArm.setDesiredNode(4);
    }));

    // mid cube
    conNumpad.btn_7.onTrue(Commands.runOnce(() -> {
      subArm.setDesiredNode(5);
    }));

    // high cube
    conNumpad.btn_4.onTrue(Commands.runOnce(() -> {
      subArm.setDesiredNode(2);
    }));
    // high cone
    conNumpad.btn_6.onTrue(Commands.runOnce(() -> {
      subArm.setDesiredNode(1);
    }));

  }

  public static boolean isPracticeBot() {
    return !pracBotSwitch.get();
  }

  public void setOpenLoop() {
    subDrivetrain.isDriveOpenLoop = true;
    subDrivetrain.configure();
  }

  public void setClosedLoop() {
    subDrivetrain.isDriveOpenLoop = false;
    subDrivetrain.configure();
  }

  private void configureAutoSelector() {
    autoChooser.setDefaultOption("null", null);

    autoChooser.addOption("Cube Then Mobility Cable", new CubeThenMobilityCable(subDrivetrain, subIntake, subArm));
    autoChooser.addOption("Cube Then Dock", new CubeThenDock(subDrivetrain, subIntake, subArm));
    autoChooser.addOption("Center Cube (NO DOCK)", new CenterCube(subDrivetrain, subIntake, subArm));
    autoChooser.addOption("Cube Then Mobility Open", new CubeThenMobilityOpen(subDrivetrain, subIntake, subArm));

    // autoChooser.addOption("Cube, Dock, Shoot", new CubeDockShoot(subDrivetrain,
    // subIntake, subArm));

    SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
