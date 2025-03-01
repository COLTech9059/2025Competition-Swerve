package frc.robot.subsystems.cage;

import frc.robot.util.RBSISubsystem;

/**
 * Uses a single motor to drive a deep cage lifting mechanism. This mechanism takes
 * inspiration from Penn State's Robot In 3 Days team
 * @author DevAspen & monkeyboy1052608
 */
public class Cage extends RBSISubsystem {

  private final CageIO io;

  public Cage(CageIO io) {
    this.io = io;
  }

  public void runMotor(double speed) {
    io.runMotor(speed);
  }

  public void stop() {
    io.stop();
  }
}
