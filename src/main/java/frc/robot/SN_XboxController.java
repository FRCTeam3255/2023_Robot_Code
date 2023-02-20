package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Custom class for Xbox Controller use
 */
public class SN_XboxController extends XboxController {

  public Trigger btn_A = new Trigger(super::getAButton);
  public Trigger btn_B = new Trigger(super::getBButton);
  public Trigger btn_X = new Trigger(super::getXButton);
  public Trigger btn_Y = new Trigger(super::getYButton);

  public Trigger btn_LeftBumper = new Trigger(super::getLeftBumper);
  public Trigger btn_RightBumper = new Trigger(super::getRightBumper);

  public Trigger btn_Start = new Trigger(super::getStartButton);
  public Trigger btn_Back = new Trigger(super::getBackButton);

  public Trigger btn_LeftStick = new Trigger(super::getLeftStickButton);
  public Trigger btn_RightStick = new Trigger(super::getRightStickButton);

  // The POV angles start at 0 in the up direction, and increase clockwise (eg
  // right is 90, upper-left is 315).
  private final int D_PAD_NORTH = 0;
  private final int D_PAD_NORTH_EAST = 45;
  private final int D_PAD_EAST = 90;
  private final int D_PAD_SOUTH_EAST = 135;
  private final int D_PAD_SOUTH = 180;
  private final int D_PAD_SOUTH_WEST = 225;
  private final int D_PAD_WEST = 270;
  private final int D_PAD_NORTH_WEST = 315;

  public Trigger btn_North = new Trigger(() -> super.getPOV() == D_PAD_NORTH);
  public Trigger btn_NorthEast = new Trigger(() -> super.getPOV() == D_PAD_NORTH_EAST);
  public Trigger btn_East = new Trigger(() -> super.getPOV() == D_PAD_EAST);
  public Trigger btn_SouthEast = new Trigger(() -> super.getPOV() == D_PAD_SOUTH_EAST);
  public Trigger btn_South = new Trigger(() -> super.getPOV() == D_PAD_SOUTH);
  public Trigger btn_SouthWest = new Trigger(() -> super.getPOV() == D_PAD_SOUTH_WEST);
  public Trigger btn_West = new Trigger(() -> super.getPOV() == D_PAD_WEST);
  public Trigger btn_NorthWest = new Trigger(() -> super.getPOV() == D_PAD_NORTH_WEST);

  private double TRIGGER_PRESS_THRESHOLD = 0.5;

  public Trigger btn_LeftTrigger = new Trigger(() -> super.getLeftTriggerAxis() > TRIGGER_PRESS_THRESHOLD);
  public Trigger btn_RightTrigger = new Trigger(() -> super.getRightTriggerAxis() > TRIGGER_PRESS_THRESHOLD);

  private double LEFT_DEADBAND = 0.1;
  private double RIGHT_DEADBAND = 0.1;

  public DoubleSupplier axis_LeftX = () -> MathUtil.applyDeadband(super.getLeftX(), LEFT_DEADBAND);
  public DoubleSupplier axis_LeftY = () -> MathUtil.applyDeadband(-super.getLeftY(), LEFT_DEADBAND);
  public DoubleSupplier axis_RightX = () -> MathUtil.applyDeadband(super.getRightX(), RIGHT_DEADBAND);
  public DoubleSupplier axis_RightY = () -> MathUtil.applyDeadband(-super.getRightY(), RIGHT_DEADBAND);
  public DoubleSupplier axis_LeftTrigger = () -> super.getLeftTriggerAxis();
  public DoubleSupplier axis_RightTrigger = () -> super.getRightTriggerAxis();

  /**
   * Set the value that the triggers must be at to register as true while being
   * used as a button. A value of zero will make the trigger act as a hair trigger
   * (will activate at the slightest touch) and a value of one will make the
   * trigger be held all the way down to activate.
   * <p>
   * Default value is 0.5
   * 
   * @param threshold for trigger button
   */
  public void setTriggerPressThreshold(double threshold) {
    TRIGGER_PRESS_THRESHOLD = MathUtil.clamp(threshold, 0, 0.99);
  }

  /**
   * Set the deadband for the left joystick.
   * <p>
   * Default value is 0.1
   * 
   * @param deadband to set for the left joystick
   */
  public void setLeftDeadband(double deadband) {
    LEFT_DEADBAND = deadband;
  }

  /**
   * Set the deadband for the right joystick.
   * <p>
   * Default value is 0.1
   * 
   * @param deadband to set for the right joystick
   */
  public void setRightDeadband(double deadband) {
    RIGHT_DEADBAND = deadband;
  }

  /**
   * Construct an instance of an SN_XboxController.
   * 
   * @param port          The port index on the Driver Station that the controller
   *                      is plugged into.
   * @param leftDeadband  The deadband for the left joystick.
   * @param rightDeadband The deadband for the right joystick.
   */
  public SN_XboxController(int port, double leftDeadband, double rightDeadband) {
    super(port);
    LEFT_DEADBAND = leftDeadband;
    RIGHT_DEADBAND = rightDeadband;
  }

  /**
   * Construct an instance of an SN_XboxController.
   *
   * @param port The port index on the Driver Station that the controller is
   *             plugged into.
   */
  public SN_XboxController(int port) {
    super(port);
  }

}