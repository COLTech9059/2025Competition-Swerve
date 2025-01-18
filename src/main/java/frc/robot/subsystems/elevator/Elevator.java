package frc.robot.subsystems.elevator;

import frc.robot.util.RBSISubsystem;

public class Elevator extends RBSISubsystem {

  public Elevator() {
    io.configureMotors();
  }

  private ElevatorIO io = new ElevatorIO();

  public void setLevel(double speed, int level) {
    io.setLevel(speed, level);
  }

  public int getLevel() {
    return io.getLevel();
  }

  public void stop() {
    io.stop();
  }

  public void activeIntake(double speed) {
    io.activeIntake(speed);
  }

  public void timedIntake(double speed, double time) {
    io.timedIntake(speed, time);
  }

  /* This will only be used if a sensor is placed in the intake */
  public void sensorIntake(double speed) {
    io.sensorIntake(speed);
  } 

  public void stopIntake() {
    io.stopIntake();
  }

  public void algaeIntake(double speed) {
    io.algaeIntake(speed);
  }

  public void timedAlgae(double speed, double time) {
    io.timedAlgae(speed, time);
  }

  public void stopAlgae() {
    io.stopAlgae();
  }

}
