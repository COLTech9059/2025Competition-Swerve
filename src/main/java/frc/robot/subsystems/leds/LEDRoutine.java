package frc.robot.subsystems.leds;

import java.util.ArrayList;

public class LEDRoutine {

  private LEDs led;
  private int[] ids;
  private ArrayList<Double> powers = new ArrayList<Double>();

  /**
   * Custom class to store an LED routine. Ids start at 1, and count up from the lowest output on
   * the Blinkin chart
   *
   * @param led the LED subsystem
   * @param delay the delay (in seconds) between pattern changes
   * @param ids an array of integers representing the patterns from the REV Blinkin chart that you
   *     want to use
   */
  public LEDRoutine(LEDs led, int[] ids) {
    this.led = led;
    this.ids = ids;

    for (int i = 0; i < ids.length; i++) {
      double num = -1.01 + ids[i] * 0.02;
      powers.add(num);
    }
  }

  /**
   * Custom class to store an LED routine. Powers start at -0.99, and count up by 0.02
   *
   * @param led The LED subsystem
   * @param delay The delay (in seconds) between pattern changes
   * @param powers An array of doubles representing the patterns from the REV Blinkin chart that you
   *     want to use
   */
  public LEDRoutine(LEDs led, double[] powers) {
    this.led = led;

    for (int i = 0; i < powers.length; i++) {
      this.powers.add(powers[i]);
    }
  }

  public double iterate(int index) {
    double value = powers.remove(index);
    led.setValue(value);
    powers.add(value);
    return value;
  }

  public ArrayList<Double> getPowers() {
    return powers;
  }
}
