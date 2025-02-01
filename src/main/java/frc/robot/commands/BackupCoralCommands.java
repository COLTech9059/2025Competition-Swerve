package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.backupCoral.BackupCoral;

public class BackupCoralCommands {

  public static Command timedPivot(BackupCoral coral, double speed, double time) {
    return Commands.run( () -> {
      coral.timedPivot(speed, time);
    },
    coral);
  }

  public static Command levelSwap(BackupCoral coral) {
    return Commands.run( () -> {
      coral.levelSwap();
    },
    coral);
  }

  public static Command timedIntake(BackupCoral coral, double speed, double time) {
    return Commands.run( () -> {
      coral.timedIntake(speed, time);
    },
    coral);
  }
}
