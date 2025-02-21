package frc.robot.subsystems.leds;

import frc.robot.util.RBSISubsystem;

public class LEDs extends RBSISubsystem {

  private final LEDsIO io;

  public LEDs(LEDsIO io) {
    this.io = io;
  }

  public void setPattern(String pattern) {
    io.setPattern(pattern);
  }

  public void setValue(double power) {
    io.setValue(power);
  }

  public void shiftColor(int shift) {
    io.shiftColor(shift);
  }
}
