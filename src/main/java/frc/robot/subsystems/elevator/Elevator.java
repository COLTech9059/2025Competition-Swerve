package frc.robot.subsystems.elevator;

import frc.robot.util.RBSISubsystem;

/**
 * This class encompasses all the hardware on the physical elevator subsystem: the elevating
 * mechanism, the coral mechanism, and the algae mechanism
 */
public class Elevator extends RBSISubsystem {

  private final ElevatorIO io;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.io.configureMotors();
  }

  /** Moves the elevator to the indicated "level" at the given speed */
  public void setLevel(double speed, int level) {
    io.setLevel(speed, level);
  }

  /** Returns the current "level" of the elevator */
  public int getLevel() {
    return io.getLevel();
  }

  /**
   * This method will return the "level" corresponding to whatever sensor is active. If no sensor is
   * detecting, it will return a -1
   */
  public int getExactLevel() {
    return io.getExactLevel();
  }

  /** Stops the elevating mechanism */
  public void stop() {
    io.stop();
  }

  /**
   * Runs the coral intake at the given speed. This method DOES NOT stop the intake motor at any
   * point
   */
  public void activeIntake(double speed) {
    io.activeIntake(speed);
  }

  /** Runs the coral intake at the given speed for the given amount of time in seconds */
  public void timedIntake(double speed, double time) {
    io.timedIntake(speed, time);
  }

  /** Currently empty; will only be used if a sensor is added to the intake */
  public void sensorIntake(double speed) {
    io.sensorIntake(speed);
  }

  /** Stops the intake motor */
  public void stopIntake() {
    io.stopIntake();
  }

  /**
   * Runs the algae intake at the given speed. This method DOES NOT stop the algae motor at any
   * point
   */
  public void algaeIntake(double speed) {
    io.algaeIntake(speed);
  }

  /** Runs the algae intake at the given speed for the given amount of time in seconds */
  public void timedAlgae(double speed, double time) {
    io.timedAlgae(speed, time);
  }

  /** Stops the algae motor */
  public void stopAlgae() {
    io.stopAlgae();
  }

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

  public void runToSensor(double speed, boolean top) {
    io.runToSensor(speed, top);
  }
}
