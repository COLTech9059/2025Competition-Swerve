package frc.robot.subsystems.leds;

import java.util.ArrayList;

/**
 * Stores an array of pattern ids from the Blinkin pattern chart and facilitates the usage of said
 * patterns in a full routine
 *
 * @author DevAspen
 */
public class LEDRoutine {

  private LEDs led;
  private ArrayList<Double> powers = new ArrayList<Double>();
  private double length = 0;

  /**
   * Custom class to store an LED routine. Ids start at 1, and count up from the lowest output on
   * the Blinkin chart
   *
   * @param led the LED subsystem
   * @param ids an array of integers representing the patterns from the REV Blinkin chart that you
   *     want to use
   */
  public LEDRoutine(LEDs led, int[] ids) {
    this.led = led;
    length = ids.length;

    for (int i = 0; i < ids.length; i++) {
      double num = -1.01 + ids[i] * 0.02;
      powers.add(num);
    }
  }

  /**
   * Custom class to store an LED routine. Powers start at -0.99, and count up by 0.02
   *
   * @param led The LED subsystem
   * @param powers An array of doubles representing the patterns from the REV Blinkin chart that you
   *     want to use
   */
  public LEDRoutine(LEDs led, double[] powers) {
    this.led = led;
    length = powers.length;

    for (int i = 0; i < powers.length; i++) {
      this.powers.add(powers[i]);
    }
  }

  /**
   * Cycles the array of powers and sets the LEDs to the next value in the array
   *
   * @param index The index to start at. (NOTE: If this value is anything other than 0, it will
   *     disrupt the initial order of patterns)
   * @return The value that was selected for iteration
   */
  public double iterate(int index) {
    if (index > length || index < 0) {
      return 0;
    }
    double value = powers.remove(index);
    led.setPattern(value);
    powers.add(value);
    return value;
  }

  /**
   * Returns the array list containing the Blinkin pattern values
   *
   * @return An ArrayList<Double> object
   */
  public ArrayList<Double> getPowers() {
    return powers;
  }
}
