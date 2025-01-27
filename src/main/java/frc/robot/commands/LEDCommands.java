package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.leds.LEDRoutine;
import frc.robot.subsystems.leds.LEDs;

public class LEDCommands {

  /**
   * Set the LEDs to a random one of the solid color routines
   *
   * @param led the LED subsystem
   * @return the relevant code statements as a Command object
   */
  public static Command randomColor(LEDs led) {
    return Commands.runOnce(
        () -> {
          led.shiftColor((int) (Math.random() * 21));
        },
        led);
  }

  /**
   * Set the LEDs to a specific pattern
   *
   * @param led the LED subsystem
   * @param power the motor value (decimal from -0.99 to 0.99) for the Blinkin to interpret
   * @return the relevant code statements as a Command object
   */
  public static Command runPattern(LEDs led, double power) {
    return Commands.runOnce(
        () -> {
          led.setValue(power);
        },
        led);
  }

  /**
   * Apply a custom LED routine using the team color patterns
   *
   * @param led the LED subsystem
   * @param routine the ID of the routine you want (pulled from a switch case)
   * @return the relevant code statements as a Command object
   */
  public static Command teamColorRoutine(LEDs led, int routine) {
    return Commands.run(
        () -> {
          Timer time = new Timer();
          time.start();
          switch (routine) {
            case 1:
              time.reset();
              if (time.get() < 2) led.setValue(0.37);
              if (time.get() > 2 && time.get() < 4) led.setValue(0.39);
              if (time.get() > 4 && time.get() < 6) led.setValue(0.41);
              if (time.get() > 6 && time.get() < 8) led.setValue(0.45);
              if (time.get() > 8 && time.get() < 10) led.setValue(0.51);
              if (time.get() > 10 && time.get() < 12) led.setValue(0.53);
              if (time.get() > 12 && time.get() < 14) led.setValue(0.55);
              if (time.get() > 14) time.reset();
              break;
            case 2:
              time.reset();
              if (time.get() < 5) led.setValue(0.37);
              if (time.get() > 5 && time.get() < 10) led.setValue(0.39);
              if (time.get() > 10) time.reset();
              break;
            case 3:
              time.reset();
              if (time.get() < 4) led.setValue(0.45);
              if (time.get() > 4 && time.get() < 8) led.setValue(0.41);
              if (time.get() > 8 && time.get() < 12) led.setValue(0.47);
              if (time.get() > 12) time.reset();
              break;
            case 4:
              time.reset();
              if (time.get() < 3) led.setValue(0.53);
              if (time.get() > 3 && time.get() < 6) led.setValue(0.55);
              if (time.get() > 6 && time.get() < 9) led.setValue(0.41);
              if (time.get() > 9 && time.get() < 12) led.setValue(0.45);
              if (time.get() > 12) time.reset();
          }
        },
        led);
  }

  /**
   * Apply a custom LED routine using the palette patterns
   *
   * @param led the LED subsystem
   * @param routine the ID of the routine you want (pulled from a switch case)
   * @return the relevant code statements as a Command object
   */
  public static Command patternRoutine(LEDs led, int routine) {
    return Commands.run(
        () -> {
          Timer time = new Timer();
          time.start();
          switch (routine) {
            case 1:
              time.reset();
              if (time.get() < 5) led.setValue(-0.99);
              if (time.get() > 5 && time.get() < 10) led.setValue(-0.97);
              if (time.get() > 10 && time.get() < 15) led.setValue(-0.89);
              if (time.get() > 15 && time.get() < 20) led.setValue(-0.79);
              if (time.get() > 20) time.reset();
          }
        },
        led);
  }

  public static Command runRoutine(LEDs led, LEDRoutine routine, double delay) {
    return Commands.run(
        () -> {
          Timer time = new Timer();
          if (time.get() >= delay) {
            routine.iterate(0);
            time.reset();
            time.start();
          }
        },
        led);
  }
}
