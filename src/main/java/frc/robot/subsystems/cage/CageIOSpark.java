package frc.robot.subsystems.cage;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;

public class CageIOSpark extends CageIO {

  private SparkMax cageMotor = new SparkMax(0, null);

  private Timer timer = new Timer();

  @Override
  public void run(double speed) {
      cageMotor.set(speed);
  }
  @Override
  public void rotate(double speed, double time) {

    timer.reset();
    timer.start();
    
    cageMotor.set(speed);

    if (timer.get() > time) {
      cageMotor.stopMotor();
    }
  }

  @Override
  public void stop() {
    cageMotor.stopMotor();
  }
}
