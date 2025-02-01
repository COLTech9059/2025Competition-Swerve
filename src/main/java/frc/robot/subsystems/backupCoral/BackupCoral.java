package frc.robot.subsystems.backupCoral;

import frc.robot.util.RBSISubsystem;

/** This is a subsystem designed as a backup in case the main elevator isn't finished or doesn't work */
public class BackupCoral extends RBSISubsystem {

  private BackupCoralIO io = new BackupCoralIO();

  public void timedPivot(double speed, double time) {
    io.timedPivot(speed, time);
  }

  public void sensorPivot(double speed) {
    io.sensorPivot(speed);
  }

  public void timedIntake(double speed, double time) {
    io.timedIntake(speed, time);
  }

  public void levelSwap() {
    io.levelSwap();
  }
}
