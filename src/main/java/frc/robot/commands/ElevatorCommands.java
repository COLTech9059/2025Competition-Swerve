package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.LEDs;

/**
 * Contains all the commands used to interface with an elevator subsystem.
 *
 * @author DevAspen & SomnolentStone
 */
public class ElevatorCommands {

  /**
   * Moves the elevator up one "level"
   *
   * @param elevator the elevator subsystem
   * @param speed the speed (as a decimal percentage) that the elevator will raise itself
   * @return the relevant code statements as a Command object
   */
  public static Command upLevel(Elevator elevator, double speed) {
    return Commands.run(() -> elevator.setLevel(speed, elevator.getLevel() + 1), elevator);
  }

  public static Command pivot(Elevator elevator, double speed, double time) {
    return Commands.sequence(
        Commands.runOnce(() -> elevator.pivot(speed)),
        Commands.waitSeconds(time),
        Commands.runOnce(() -> elevator.stopPivot()));
  }

  /**
   * Moves the elevator down one "level"
   *
   * @param elevator the elevator subsystem
   * @param speed the speed (as a decimal percentage) that the elevator will lower itself
   * @return the relevant code statements as a Command object
   */
  public static Command downLevel(Elevator elevator, double speed) {
    return Commands.run(() -> elevator.setLevel(speed, elevator.getLevel() - 1), elevator);
  }

  /**
   * Runs the coral intake for a specified speed and time
   *
   * @param elevator the elevator subsystem
   * @param speed the speed (as a decimal percentage) at which the intake motor will run
   * @param time the amount of time (in seconds) that the motor will run for
   * @return the relevant code statements as a Command object
   */
  public static Command timedIntake(Elevator elevator, double speed, double time) {
    return Commands.run(() -> elevator.timedIntake(speed, time), elevator);
  }

  public static Command timedOuttake(
      Elevator elevator,
      double pivotSpeed,
      double outtakeSpeed,
      double pivotTime,
      double outtakeTime) {
    pivotSpeed = Math.abs(pivotSpeed);
    outtakeSpeed = Math.abs(outtakeSpeed);
    return Commands.sequence(
        pivot(elevator, -pivotSpeed, pivotTime), timedIntake(elevator, -outtakeSpeed, outtakeTime));
  }

  /**
   * Scores a coral gamepiece at the specified level, then goes down to level 0
   *
   * @param elevator The elevator subsystem
   * @param led The LED subsystem
   * @param speed The speed (as a decimal percentage) at which the elevator will move
   * @param level The "level" the elevator will move to
   * @param outtakeSpeed The speed (as a decimal percentage) at which the intake motor will run
   * @param outtakeTime The amount of time (in seconds) that the intake motor will run for
   * @return The relevant code statements as a Command object
   */
  public static Command coralScore(
      Elevator elevator,
      LEDs led,
      double speed,
      int level,
      double outtakeSpeed,
      double outtakeTime) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDCommands.runPattern(led, 0.31)),
        Commands.run(() -> elevator.setLevel(speed, level)),
        Commands.runOnce(() -> LEDCommands.runPattern(led, 0.27)),
        Commands.run(() -> elevator.timedIntake(-Math.abs(outtakeSpeed), outtakeTime)),
        Commands.runOnce(() -> LEDCommands.runPattern(led, 0.29)),
        Commands.run(() -> elevator.setLevel(speed, 0)),
        Commands.runOnce(() -> LEDCommands.interrupt(led)));
  }

  /**
   * Collects a coral gamepiece from the supply station
   *
   * @param elevator the elevator subsystem
   * @param led The LED subsystem
   * @param speed the speed (as a decimal percentage) at which the elevator will move
   * @param intakeSpeed the speed (as a decimal percentage) at which the intake motor will run
   * @param intakeTime the amount of time (in seconds) that the intake motor will run for
   * @return the relevant code statements as a Command object
   */
  public static Command coralCollect(
      Elevator elevator, LEDs led, double speed, double intakeSpeed, double intakeTime) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDCommands.runPattern(led, 0.31)),
        Commands.run(() -> elevator.setLevel(speed, 2)),
        Commands.runOnce(() -> LEDCommands.runPattern(led, 0.27)),
        Commands.run(() -> elevator.timedIntake(intakeSpeed, intakeTime)),
        Commands.runOnce(() -> LEDCommands.runPattern(led, 0.29)),
        Commands.run(() -> elevator.setLevel(speed, 1)),
        Commands.runOnce(() -> LEDCommands.interrupt(led)));
  }

  // TEMPORARY; can comment out if you want (TODO: temporary, comment out or delete if you don't
  // want it.)
  public static Command moveElevator(Elevator elevator, double speed) {
    return Commands.run(
        () -> {
          elevator.runMotor(speed);
        },
        elevator);
  }

  /**
   * Run a timed test of the first stage motor
   *
   * @param elevator The elevator subsystem
   * @param led The LED subsystem
   * @param speed The speed to run the motor (as a decimal percentage)
   * @param time The time in seconds to run the motor
   * @return The relevant code statements as a Command object
   */
  public static Command oneTest(Elevator elevator, LEDs led, double speed, double time) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDCommands.runPattern(led, 0.31)),
        Commands.runOnce(() -> elevator.runMotor(speed), elevator),
        Commands.waitSeconds(time),
        Commands.runOnce(elevator::stop, elevator),
        Commands.runOnce(() -> LEDCommands.interrupt(led)));
  }

  /**
   * Set the first stage motor to run without stopping
   *
   * @param elevator The elevator subsystem
   * @param led The LED subsystem
   * @param speed The speed to run the motor (as a decimal percentage)
   * @return The relevant code statements as a Command object
   */
  public static Command runWithoutStop(Elevator elevator, LEDs led, double speed) {
    return Commands.startEnd(
        () -> {
          LEDCommands.runPattern(led, 0.31);
          elevator.runMotor(speed);
        },
        () -> LEDCommands.interrupt(led),
        elevator);
  }

  /**
   * Stop the movement of the elevator, unconditionally
   *
   * @param elevator The elevator subsystem
   * @return The relevant code statements as a Command object
   */
  public static Command stopElevator(Elevator elevator) {
    return Commands.runOnce(elevator::stop, elevator);
  }

  public static void interrupt(Elevator elevator) {
    CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().requiring(elevator));
  }

  /**
   * Runs the stage one motor until a sensor is triggered
   *
   * @param elevator The elevator subsystem
   * @param led The LED subsystem
   * @param speed The speed to run the motor (as a decimal percentage)
   * @return The relevant code statements as a Command object
   */
  public static Command runToSensor(Elevator elevator, LEDs led, double speed) {
    return Commands.runEnd(
        () -> {
          elevator.runMotor(speed);
          if (elevator.getSwitch()) interrupt(elevator);
        },
        () -> LEDCommands.interrupt(led),
        elevator);
  }
}
