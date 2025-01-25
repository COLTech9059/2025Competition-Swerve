package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {

  public static Command upLevel(Elevator elevator, double speed) {
    return Commands.run(
        () -> {
          elevator.setLevel(speed, elevator.getLevel() + 1);
        },
        elevator);
  }

  public static Command downLevel(Elevator elevator, double speed) {
    return Commands.run(
        () -> {
          elevator.setLevel(speed, elevator.getLevel() - 1);
        },
        elevator);
  }

  public static Command timedIntake(Elevator elevator, double speed, double time) {
    return Commands.run(
        () -> {
          elevator.timedIntake(speed, time);
        },
        elevator);
  }

  public static Command timedAlgae(Elevator elevator, double speed, double time) {
    return Commands.run(
        () -> {
          elevator.timedAlgae(speed, time);
        },
        elevator);
  }

  public static Command coralScore(Elevator elevator, double speed, int level, double outtakeSpeed, double outtakeTime) {
    return Commands.run(
        () -> {
          elevator.setLevel(speed, level);
          elevator.timedIntake(-outtakeSpeed, outtakeTime);
          elevator.setLevel(speed, 1);
        },
        elevator);
  }

  public static Command coralCollect(Elevator elevator, double speed, double intakeSpeed, double intakeTime) {
    return Commands.run(
        () -> {
          elevator.setLevel(speed, 2);
          elevator.timedIntake(intakeSpeed, intakeTime);
          elevator.setLevel(speed, 1);
        },
        elevator);
  }
}
