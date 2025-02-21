package frc.robot.subsystems.leds;

import frc.robot.util.RBSISubsystem;

/**
 * A subsystem which interfaces with LEDs
 * @author DevAspen
 */
public class LEDs extends RBSISubsystem {

  private final LEDsIO io;

  /** 
   * Constructs an LED object, which interfaces with an LED strip on the robot
   * @param io The LEDsIO object for the subsystem to interface with (The hardware class to use)
  */
  public LEDs(LEDsIO io) {
    this.io = io;
  }

  /**
   * Sets the LEDs to a specific pattern from the Blinkin chart
   * @param pattern The name of the pattern you want (this is pulled from a switch case, so there is limited options)
   */
  public void setPattern(String pattern) {
    io.setPattern(pattern);
  }

  /**
   * Sets the LEDs to a specific pattern from the Blinkin chart
   * @param power The motor power to supply to the Blinkin (ex: -0.99, -0.15, 0.37, 0.99)
   */
  public void setPattern(double power) {
    io.setPattern(power);
  }

  /**
   * Set the LEDs to a solid color pattern, specified by an integer shift from the first solid color pattern in the list
   * @param shift Integer shift value to apply. Check the Blinkin pattern chart to see what the pattern will be)
   */
  public void shiftColor(int shift) {
    io.shiftColor(shift);
  }
}
