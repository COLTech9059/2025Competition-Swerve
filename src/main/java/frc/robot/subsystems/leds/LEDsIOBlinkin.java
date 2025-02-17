package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDsIOBlinkin extends LEDsIO {

  private Spark blinkin = new Spark(0);

  @Override
  public void setPattern(String pattern) {
    switch (pattern) {
      case "rainbow":
        blinkin.set(-0.99);
        break;
      case "confetti":
        blinkin.set(-0.87);
        break;
      case "shot":
        blinkin.set(-0.85);
        break;
      case "sinelon":
        blinkin.set(-0.79);
        break;
      case "beatsPerMinute":
        blinkin.set(-0.69);
        break;
      case "fire":
        blinkin.set(-0.59);
        break;
      case "twinkles":
        blinkin.set(-0.55);
        break;
      case "colorWaves":
        blinkin.set(-0.45);
        break;
      case "larsonScanner":
        blinkin.set(-0.35);
        break;
      case "lightChase":
        blinkin.set(-0.31);
        break;
      case "heartbeat":
        blinkin.set(-0.25);
        break;
      case "breath":
        blinkin.set(-0.17);
        break;
      case "strobe":
        blinkin.set(-0.11);
        break;
      case "hotPink":
        blinkin.set(0.57);
        break;
      case "darkRed":
        blinkin.set(0.59);
        break;
      case "red":
        blinkin.set(0.61);
        break;
      case "redOrange":
        blinkin.set(0.63);
        break;
      case "orange":
        blinkin.set(0.65);
        break;
      case "gold":
        blinkin.set(0.67);
        break;
      case "yellow":
        blinkin.set(0.69);
        break;
      case "lawnGreen":
        blinkin.set(0.71);
        break;
      case "lime":
        blinkin.set(0.73);
        break;
      case "darkGreen":
        blinkin.set(0.75);
        break;
      case "green":
        blinkin.set(0.77);
        break;
      case "blueGreen":
        blinkin.set(0.79);
        break;
      case "aqua":
        blinkin.set(0.81);
        break;
      case "skyBlue":
        blinkin.set(0.83);
        break;
      case "darkBlue":
        blinkin.set(0.85);
        break;
      case "blue":
        blinkin.set(0.87);
        break;
      case "blueViolet":
        blinkin.set(0.89);
        break;
      case "violet":
        blinkin.set(0.91);
        break;
      case "white":
        blinkin.set(0.93);
        break;
      case "gray":
        blinkin.set(0.95);
        break;
      case "darkGray":
        blinkin.set(0.97);
        break;
      default:
        blinkin.set(0.99);
        break;
    }
  }

  @Override
  public void setValue(double power) {
    blinkin.set(power);
  }

  @Override
  public void shiftColor(int shift) {
    double colorDouble = shift;
    while (colorDouble >= 21) {
      colorDouble -= 21;
    }
    colorDouble *= 0.02;
    colorDouble += 0.57;
    blinkin.set(colorDouble);
  }
}
