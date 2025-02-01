package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.cage.Cage;

public class CageCommands {
  
  public static Command timedRun(Cage cage, double speed, double time) {
    return Commands.run( () -> {
      cage.rotate(speed, time);
    },
    cage);
  }

  /* Create an instant Command ( using Commads.runOnce() ) that will set the cage to run, but not stop */
  public static Command run(Cage cage, double speed) {
    return Commands.runOnce( () -> {
      cage.run(speed);
    },
    cage);
  }

  /* Create an instant Command to do the same thing, but stop the cage instead */
  public static Command stopCage(Cage cage, double speed) {
    return Commands.runOnce( () -> {
      cage.stop();
    },
    cage);
  }

}