// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;
import com.frcteam3255.joystick.SN_SwitchboardStick;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.SuperShuffle;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constLEDs;
import frc.robot.Constants.constArm.ArmHeight;
import frc.robot.Constants.constArm.ArmState;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefCollector;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeCubeDeploy;
import frc.robot.commands.IntakeCubeRetract;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.commands.MoveArm;
import frc.robot.commands.PivotCollector;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.SetLEDs;
import frc.robot.commands.SetRumble;
import frc.robot.commands.Auto.OnePiece.CenterCube;
import frc.robot.commands.Auto.OnePiece.CubeThenEngageCenter;
import frc.robot.commands.Auto.OnePiece.CubeThenEngageOpen;
import frc.robot.commands.Auto.OnePiece.CubeThenMobilityCable;
import frc.robot.commands.Auto.OnePiece.CubeThenMobilityOpen;
import frc.robot.commands.Auto.TwoPiece.TwoCubeDockOpen;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  ShuffleboardTab autoTab = Shuffleboard.getTab("SuperShuffle");

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);
  private final SN_SwitchboardStick conSwitchboard = new SN_SwitchboardStick(mapControllers.SWITCHBOARD_USB);
  private final SN_SwitchboardStick conNumpad = new SN_SwitchboardStick(mapControllers.NUMPAD_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Arm subArm = new Arm();
  private final SuperShuffle subSuperShuffle = new SuperShuffle(subArm);
  private final Intake subIntake = new Intake();
  private final Collector subCollector = new Collector();
  private final Vision subVision = new Vision();
  private final LEDs subLEDs = new LEDs();

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  private static DigitalInput pracBotSwitch = new DigitalInput(9);
  private final Trigger teleopTrigger = new Trigger(() -> RobotState.isEnabled() && RobotState.isTeleop());
  private final Trigger hasGamePieceTrigger = new Trigger(() -> subIntake.isGamePieceCollected());

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
    subArm.setDefaultCommand(new MoveArm(subArm, subCollector, conOperator.axis_LeftY, conOperator.axis_RightY));
    subIntake.setDefaultCommand(subIntake.holdCommand());
    subVision.setDefaultCommand(new AddVisionMeasurement(subDrivetrain,
        subVision));
    subLEDs.setDefaultCommand(new SetLEDs(subLEDs, subIntake, subDrivetrain, subArm));

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

    // I really hope this never gets used (reset arm arm motor encoders)
    conDriver.btn_Start
        .onTrue(Commands.runOnce(() -> subArm.resetJointEncodersToAbsolute()));

    // while true do robot oriented, default to field oriented
    conDriver.btn_LeftBumper
        .whileTrue(Commands.runOnce(() -> subDrivetrain.setRobotRelative()))
        .onFalse(Commands.runOnce(() -> subDrivetrain.setFieldRelative()));

    conDriver.btn_RightBumper
        .whileTrue(Commands.run(() -> subDrivetrain.setDefenseMode(), subDrivetrain))
        .whileTrue(Commands.run(() -> subLEDs.setLEDPattern(constLEDs.DEFENSE_MODE_COLOR)));

    // Operator

    // Order of buttons: set height, prep, place

    // Intake Cube (LB)
    conOperator.btn_LeftBumper
        .onTrue(new IntakeCubeDeploy(subArm, subCollector, subIntake))
        .onFalse(new IntakeCubeRetract(subArm, subCollector, subIntake));

    // Prep Cube (X)
    conOperator.btn_X
        .onTrue(subArm.setDesiredGamePiece(GamePiece.CUBE))
        .onTrue(subArm.prepPlaceCommand());

    // Place Cube (LT)
    conOperator.btn_LeftTrigger.whileTrue(subIntake.releaseCommand());

    // Intake Cone (RB)
    conOperator.btn_RightBumper
        .onTrue(subArm.intakeFloorDeployCommand())
        .whileFalse(subArm.intakeFloorStowCommand())
        .whileTrue(new IntakeGamePiece(subIntake));

    // Prep Cone (B)
    conOperator.btn_B
        .onTrue(subArm.setDesiredGamePiece(GamePiece.CONE))
        .onTrue(subArm.prepPlaceCommand());

    // Place Cone (RT)
    conOperator.btn_RightTrigger.whileTrue(new PlaceGamePiece(subArm, subIntake));

    // Set stow Arm preset (a)
    conOperator.btn_A.onTrue(subArm.stowCommand());

    // Set Shelf Arm preset (y)
    conOperator.btn_Y.onTrue(subArm.stateFromStowCommand(ArmState.SHELF_INTAKE))
        .whileTrue(new IntakeGamePiece(subIntake));

    // Spin the Intake forward
    conOperator.btn_Start
        .whileTrue(new IntakeGamePiece(subIntake));

    // Spin the Intake in reverse (back)
    conOperator.btn_Back
        .whileTrue(subIntake.releaseCommand());

    conOperator.btn_North.onTrue(subArm.setDesiredArmHeight(ArmHeight.LOW));
    conOperator.btn_East.onTrue(subArm.setDesiredArmHeight(ArmHeight.MID));
    conOperator.btn_West.onTrue(subArm.setDesiredArmHeight(ArmHeight.MID));
    conOperator.btn_South.onTrue(subArm.setDesiredArmHeight(ArmHeight.HIGH));

    // teleopTrigger.onTrue(new SetRumble(conDriver, conOperator, subIntake));
  }

  public void saveNodeState(int chosenGrid,
      int betweenStart_1, int betweenEnd_1, int addAmount_1,
      int betweenStart_2, int betweenEnd_2, int addAmount_2) {

    subArm.setGridChoice(chosenGrid);

    if (subArm.getDesiredNode() >= betweenStart_1 && subArm.getDesiredNode() <= betweenEnd_1) {
      subArm.setDesiredNode(subArm.getDesiredNode() + addAmount_1);
    }

    if (subArm.getDesiredNode() >= betweenStart_2 && subArm.getDesiredNode() <= betweenEnd_2) {
      subArm.setDesiredNode(subArm.getDesiredNode() + addAmount_2);
    }
  }

  public static boolean isPracticeBot() {
    return !pracBotSwitch.get();
  }

  private void configureAutoSelector() {
    autoChooser.setDefaultOption("null", null);

    autoChooser.addOption("Score Cube Then Mobility Cable",
        new CubeThenMobilityCable(subDrivetrain, subIntake, subArm));
    autoChooser.addOption("Score Cube Then Engage Center", new CubeThenEngageCenter(subDrivetrain, subIntake, subArm));
    autoChooser.addOption("Score Cube Center (NO DOCK)", new CenterCube(subDrivetrain, subIntake, subArm));
    autoChooser.addOption("Score Cube Then Mobility Open", new CubeThenMobilityOpen(subDrivetrain, subIntake, subArm));
    // autoChooser.addOption("Score Cube Then Engage Open", new
    // CubeThenEngageOpen(subDrivetrain, subIntake, subArm));

    // autoChooser.addOption("Score TWO Cubes Then Engage Open - DO NOT USE",
    // new TwoCubeDockOpen(subDrivetrain, subIntake, subArm));

    SmartDashboard.putData(autoChooser);

    autoTab
        .add("Auto Chooser", autoChooser)
        .withSize(2, 1)
        .withPosition(7, 2);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
