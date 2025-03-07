package frc.robot.subsystems.cage;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class CageIOSpark extends CageIO {

  private SparkMax cage = new SparkMax(Constants.cageID, MotorType.kBrushless);

  @Override
  public void runMotor(double speed) {
    cage.set(speed);
  }

  @Override
  public void stop() {
    cage.stopMotor();
  }
}
