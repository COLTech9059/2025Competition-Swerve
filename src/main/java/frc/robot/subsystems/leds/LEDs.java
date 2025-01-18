package frc.robot.subsystems.leds;

import frc.robot.util.RBSISubsystem;

public class LEDs extends RBSISubsystem {
  private LEDsIO io = new LEDsIO();

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
