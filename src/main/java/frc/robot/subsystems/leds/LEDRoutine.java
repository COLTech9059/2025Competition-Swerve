package frc.robot.subsystems.leds;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDRoutine {

  private LEDs led;
  private double delay;
  private int[] ids;
  private ArrayList<Double> powers = new ArrayList<Double>();

  /**
   * Custom class to store an LED routine. Ids start at 0, and count up from the lowest output on the Blinkin chart
   * @param led the LED subsystem
   * @param delay the delay (in seconds) between pattern changes
   * @param ids an array of integers representing the patterns from the REV Blinkin chart that you want to use
   */
  public LEDRoutine(LEDs led, double delay, int[] ids) {
    this.led = led;
    this.delay = delay;
    this.ids = ids;
    
    for(int i = 0; i < ids.length; i++) {
      double num = -0.99 + ids[i] * 0.02;
      powers.add(num);
    }
  }

  public double iterate(int index) {
    double value = powers.remove(index);
    led.setValue(value);
    powers.add(value);
    return value;
  }

  public ArrayList<Double> getArray() {
    return powers;
  }
}
