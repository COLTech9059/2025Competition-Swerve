package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;

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

  /**
   * Scores a coral gamepiece at the specified level, then goes down to level 0
   *
   * @param elevator the elevator subsystem
   * @param speed the speed (as a decimal percentage) at which the elevator will move
   * @param level the "level" the elevator will move to
   * @param outtakeSpeed the speed (as a decimal percentage) at which the intake motor will run
   * @param outtakeTime the amount of time (in seconds) that the intake motor will run for
   * @return the relevant code statements as a Command object
   */
  public static Command coralScore(
      Elevator elevator, double speed, int level, double outtakeSpeed, double outtakeTime) {
    return Commands.sequence(
        Commands.run(() -> elevator.setLevel(speed, level)),
        Commands.run(() -> elevator.timedIntake(-Math.abs(outtakeSpeed), outtakeTime)),
        Commands.run(() -> elevator.setLevel(speed, 0)));
  }

  /**
   * Collects a coral gamepiece from the supply station
   *
   * @param elevator the elevator subsystem
   * @param speed the speed (as a decimal percentage) at which the elevator will move
   * @param intakeSpeed the speed (as a decimal percentage) at which the intake motor will run
   * @param intakeTime the amount of time (in seconds) that the intake motor will run for
   * @return the relevant code statements as a Command object
   */
  public static Command coralCollect(
      Elevator elevator, double speed, double intakeSpeed, double intakeTime) {
    return Commands.run(
        () -> {
          elevator.setLevel(speed, 2);
          elevator.timedIntake(intakeSpeed, intakeTime);
          elevator.setLevel(speed, 1);
        },
        elevator);
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
   * @param speed The speed to run the motor (as a decimal percentage)
   * @param time The time in seconds to run the motor
   * @return The relevant code statements as a Command object
   */
  public static Command oneTest(Elevator elevator, double speed, double time) {
    return Commands.sequence(
        Commands.runOnce(() -> elevator.runMotor(speed), elevator),
        Commands.waitSeconds(time),
        Commands.runOnce(elevator::stop, elevator));
  }

  /**
   * Set the first stage motor to run without stopping
   *
   * @param elevator The elevator subsystem
   * @param speed The speed to run the motor (as a decimal percentage)
   * @return The relevant code statements as a Command object
   */
  public static Command runWithoutStop(Elevator elevator, double speed) {
    return Commands.runOnce(() -> elevator.runMotor(speed), elevator);
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

  /**
   * Runs the stage one motor until a sensor is triggered
   *
   * @param elevator The elevator subsystem
   * @param speed The speed to run the motor (as a decimal percentage)
   * @param top Whether or not the program checks the top sensor
   * @return The relevant code statements as a Command object
   */
  public static Command runToSensor(Elevator elevator, double speed, boolean top) {
    return Commands.runEnd(() -> elevator.runToSensor(speed, top), () -> elevator.stop(), elevator);
  }
}
