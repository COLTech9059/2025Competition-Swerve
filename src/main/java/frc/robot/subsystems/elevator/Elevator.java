package frc.robot.subsystems.elevator;

import frc.robot.util.RBSISubsystem;

/**
 * Encompasses all the hardware on the physical elevator subsystem: the elevating mechanism, the
 * coral mechanism, and the algae mechanism
 *
 * @author DevAspen & SomnolentStone
 */
public class Elevator extends RBSISubsystem {

  private final ElevatorIO io;

  /**
   * Constructs an Elevator object, which controls a suite of game piece manipulation mechanisms on
   * the robot
   *
   * @param io The ElevatorIO object for the program to interface with
   */
  public Elevator(ElevatorIO io) {
    this.io = io;
    this.io.configureMotors();
  }

  /**
   * Moves the elevator to a specific "level"
   *
   * @param speed The speed at which to move the elevator, as a decimal percentage
   * @param level The "level" to move the elevator to
   */
  public void setLevel(double speed, int level) {
    io.setLevel(speed, level);
  }

  public void pivot(double speed) {
    io.pivot(speed);
  }

  public void stopPivot() {
    io.stopPivot();
  }

  /**
   * Gets the last reached "level" of the elevator
   *
   * @return The level of the elevator, as an integer
   */
  public int getLevel() {
    return io.getLevel();
  }

  /**
   * Gets the exact level of the elevator, for use with internal logic
   *
   * @return The level of the elevator as an integer, defaulting to -1 if it is not at any exact
   *     level
   */
  public int getExactLevel() {
    return io.getExactLevel();
  }

  /** Stops the motion of the elevator */
  public void stop() {
    io.stop();
  }

  /**
   * Runs the intake without a built-in stop
   *
   * @param speed The speed at which to run the intake motor, as a decimal percentage
   */
  public void activeIntake(double speed) {
    io.activeIntake(speed);
  }

  /**
   * Runs the intake for a set amount of time before stopping
   *
   * @param speed The speed at which to run the intake motor, as a decimal percentage
   * @param time The time to run the intake motor for, in seconds
   */
  public void timedIntake(double speed, double time) {
    io.timedIntake(speed, time);
  }

  /**
   * CURRENTLY EMPTY; Runs the intake until a sensor is triggered
   *
   * @param speed The speed at which to run the intake motor, as a decimal percentage
   */
  public void sensorIntake(double speed) {
    io.sensorIntake(speed);
  }

  /** Stops the intake motor */
  public void stopIntake() {
    io.stopIntake();
  }

  /**
   * Runs the algae intake without a built-in stop
   *
   * @param speed The speed at which to run the algae intake motor, as a decimal percentage
   */
  public void algaeIntake(double speed) {
    io.algaeIntake(speed);
  }

  /**
   * Runs the algae intake for a set amount of time before stopping
   *
   * @param speed The speed at which to run the algae intake motor, as a decimal percentage
   * @param time The time to run the algae intake for, in seconds
   */
  public void timedAlgae(double speed, double time) {
    io.timedAlgae(speed, time);
  }

  /** Stops the algae intake motor */
  public void stopAlgae() {
    io.stopAlgae();
  }

  /**
   * Temporary method for testing elevator movement
   *
   * @param speed The speed at which the elevator will move
   */
  public void runMotor(double speed) {
    io.runMotor(speed);
  }

  @Override
  public void periodic() {
    io.periodicUpdates();
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Returns the speed of the elevator */
  public double getSpeed() {
    return io.getSpeed();
  }

  /**
   * Increases the elevator speed by the given amount
   *
   * @param value The double value to increase the speed by
   */
  public void incrementSpeed(double value) {
    io.incrementSpeed(value);
  }

  /**
   * Decreases the elevator speed by the given amount
   *
   * @param value The double value to decrease the speed by
   */
  public void decrementSpeed(double value) {
    io.decrementSpeed(value);
  }

  /** Returns the status of the limit switch */
  public boolean getSwitch() {
    return io.getSwitch();
  }

  /**
   * Runs the elevator until it reaches a specific sensor
   *
   * @param speed The speed at which to run the elevator, as a decimal percentage
   */
  public void runToSensor(double speed) {
    io.runToSensor(speed);
  }
}
