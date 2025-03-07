package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.cage.Cage;
import frc.robot.subsystems.leds.LEDs;

public class CageCommands {

  public static Command timedRun(Cage cage, LEDs led, double speed, double time) {
    return Commands.sequence(
        LEDCommands.randomColor(led),
        Commands.runOnce(() -> cage.runMotor(speed), cage),
        Commands.waitSeconds(time),
        Commands.runOnce(() -> cage.stop(), cage),
        Commands.runOnce(() -> LEDCommands.interrupt(led)));
  }
}
