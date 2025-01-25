package frc.robot.subsystems.elevator;

public class ElevatorIO extends Elevator {

  public void configureMotors() {}

  public void setLevel(double speed, int level) {}

  public int getLevel() {
    return 0;
  }

  public void stop() {}

  public void activeIntake(double speed) {}

  public void timedIntake(double speed, double time) {}

  public void sensorIntake(
      double speed) {} /* This will only be used if a sensor is placed in the intake */

  public void stopIntake() {}

  public void algaeIntake(double speed) {}

  public void timedAlgae(double speed, double time) {}

  public void stopAlgae() {}
}
