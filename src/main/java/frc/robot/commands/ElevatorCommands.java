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
}
