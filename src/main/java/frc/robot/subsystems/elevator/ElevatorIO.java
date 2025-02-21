package frc.robot.subsystems.elevator;

/**
 * Architecture class which allows the use of multiple hardware classes
 * @author DevAspen & SomnolentStone
 */
public class ElevatorIO {

  /**
   * Configures the motor controller objects
   */
  public void configureMotors() {}

  /**
   * Moves the elevator to a specific "level"
   * @param speed The speed at which to move the elevator, as a decimal percentage
   * @param level The "level" to move the elevator to
   */
  public void setLevel(double speed, int level) {}

  /**
   * Gets the last reached "level" of the elevator
   * @return The level of the elevator, as an integer
   */
  public int getLevel() {
    return 0;
  }

  /**
   * Gets the exact level of the elevator, for use with internal logic
   * @return The level of the elevator as an integer, defaulting to -1 if it is not at any exact level  
   */ 
  public int getExactLevel() {
    return -1;
  }

 /**
  * Temporary method for testing elevator movement
  * @param speed The speed at which the elevator will move
  */
  public void runMotor(double speed) {}

  public void setVoltage(double volts) {}

  /**
   * Stops the motion of the elevator
   */
  public void stop() {}

  /**
   * Runs the intake without a built-in stop
   * @param speed The speed at which to run the intake motor, as a decimal percentage
   */
  public void activeIntake(double speed) {}

  /**
   * Runs the intake for a set amount of time before stopping
   * @param speed The speed at which to run the intake motor, as a decimal percentage
   * @param time The time to run the intake motor for, in seconds
   */
  public void timedIntake(double speed, double time) {}

  /**
   * CURRENTLY EMPTY; Runs the intake until a sensor is triggered
   * @param speed The speed at which to run the intake motor, as a decimal percentage
   */
  public void sensorIntake(double speed) {} 

  /**
   * Stops the intake motor
   */
  public void stopIntake() {}

  /**
   * Runs the algae intake without a built-in stop
   * @param speed The speed at which to run the algae intake motor, as a decimal percentage
   */
  public void algaeIntake(double speed) {}

  /**
   * Runs the algae intake for a set amount of time before stopping
   * @param speed The speed at which to run the algae intake motor, as a decimal percentage
   * @param time The time to run the algae intake for, in seconds
   */
  public void timedAlgae(double speed, double time) {}

  /**
   * Stops the algae intake motor
   */
  public void stopAlgae() {}

  /**
   * Runs the elevator until it reaches a specific sensor
   * @param speed The speed at which to run the elevator, as a decimal percentage
   */
  public void runToSensor(double speed) {}

  /**
   * Updates values that need to be changed every period
   */
  public void periodicUpdates() {}
}
