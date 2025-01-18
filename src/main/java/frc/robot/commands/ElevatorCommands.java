package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {

  public static Command upLevel(Elevator elevator, double speed) {
    return Commands.run( () -> {
      elevator.setLevel(speed, elevator.getLevel() + 1);
    },
    elevator);
  }

  public static Command downLevel(Elevator elevator, double speed) {
    return Commands.run( () -> {
      elevator.setLevel(speed, elevator.getLevel() - 1);
    },
    elevator);
  }

  public static Command timedIntake(Elevator elevator, double speed, double time) {
    return Commands.run( () -> {
      elevator.timedIntake(speed, time);
    },
    elevator);
  }

  public static Command timedAlgae(Elevator elevator, double speed, double time) {
    return Commands.run( () -> {
      elevator.timedAlgae(speed, time);
    },
    elevator);
  }

  

}
