package frc.robot.subsystems.elevator;

public class ElevatorIO {

  public void configureMotors() {}

  public void setLevel(double speed, int level) {}

  public int getLevel() {
    return 0;
  }

  // TEMPORARY; comment out or delete when done with use.
  public void runMotor(double speed) {}

  public void stop() {}

  public void activeIntake(double speed) {}

  public void timedIntake(double speed, double time) {}

  public void sensorIntake(
      double speed) {} /* This will only be used if a sensor is placed in the intake */

  public void stopIntake() {}

  public void algaeIntake(double speed) {}

  public void timedAlgae(double speed, double time) {}

  public void stopAlgae() {}

  public void encoderUpdates() {}
}
