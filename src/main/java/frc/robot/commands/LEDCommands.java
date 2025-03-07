package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.leds.LEDRoutine;
import frc.robot.subsystems.leds.LEDs;

/**
 * Contains all the commands used to interface with an LEDs subsystem
 *
 * @author DevAspen
 */
public class LEDCommands {

  /**
   * Set the LEDs to a random one of the solid color routines
   *
   * @param led the LED subsystem
   * @return the relevant code statements as a Command object
   */
  public static Command randomColor(LEDs led) {
    return Commands.runOnce(() -> led.shiftColor((int) (Math.random() * 21) + 1), led);
  }

  public static Command setColor(LEDs led, String color) {
    return Commands.run(() -> led.setPattern(color));
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
          led.setPattern(power);
        },
        led);
  }

  /**
   * Apply a custom LED routine using the team color patterns
   *
   * @param led the LED subsystem
   * @param routineID the ID of the routine you want (pulled from a switch case)
   * @return the relevant code statements as a Command object
   */
  public static Command teamColorRoutine(LEDs led, int routineID) {
    LEDRoutine routine;
    switch (routineID) {
      case 1:
        routine = new LEDRoutine(led, new double[] {0.37, 0.39, 0.41, 0.45, 0.51, 0.53, 0.55});
        return runRoutine(led, routine, 2);
      case 2:
        routine = new LEDRoutine(led, new double[] {0.37, 0.39});
        return runRoutine(led, routine, 5);
      case 3:
        routine = new LEDRoutine(led, new double[] {0.45, 0.41, 0.47});
        return runRoutine(led, routine, 4);
      case 4:
        routine = new LEDRoutine(led, new double[] {0.53, 0.55, 0.41, 0.45});
        return runRoutine(led, routine, 3);
      default:
        return Commands.runOnce(
            () -> DriverStation.reportWarning("Invalid LED routine selected", false));
    }
  }

  /**
   * Apply a custom LED routine using the palette patterns
   *
   * @param led the LED subsystem
   * @param routineID the ID of the routine you want (pulled from a switch case)
   * @return the relevant code statements as a Command object
   */
  public static Command patternRoutine(LEDs led, int routineID) {
    LEDRoutine routine;

    switch (routineID) {
      case 1:
        routine = new LEDRoutine(led, new double[] {-0.99, -0.97, -0.89, -0.79});
        return runRoutine(led, routine, 5);
      default:
        return Commands.runOnce(
            () -> DriverStation.reportWarning("Invalid LED routine selected", false));
    }
  }

  /**
   * Runs the pattern of an LEDRoutine object with a delay between pattern cycles
   *
   * @param led The LED subsystem
   * @param routine The LEDRoutine to run
   * @param delay The delay between pattern changes
   * @return The relevant code statements as a Command object
   */
  public static Command runRoutine(LEDs led, LEDRoutine routine, double delay) {
    Timer time = new Timer();
    time.reset();
    time.start();

    return Commands.run(
        () -> {
          if (time.get() >= delay) {
            routine.iterate(0);
            time.reset();
            time.start();
          }
        },
        led);
  }

  public static Command setIdlePattern(LEDs led, int id) {
    if (id > 78) id = 1;
    double power = -1.01 + 0.02 * id;
    return Commands.runOnce(() -> led.setDefaultCommand(runPattern(led, power)));
  }

  public static Command interrupt(LEDs led) {
    CommandScheduler cmd = CommandScheduler.getInstance();
    return Commands.runOnce(() -> cmd.cancel(cmd.requiring(led)));
  }
}
