package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ElevatorIOSparkTest extends ElevatorIO {

  // Define motor objects
  private SparkMax eMotor = new SparkMax(Constants.eMotorID, MotorType.kBrushless);
  private RelativeEncoder eEncoder = eMotor.getEncoder();
  private SparkMaxConfig eConfig;

  @Override
  public void configureMotors() {
    eConfig.inverted(false);
    eConfig.openLoopRampRate(0.2);
    eConfig.closedLoopRampRate(0.2);
  }

  @Override
  public void runMotor(double speed) {
    eMotor.set(speed);
  }

  @Override
  public void stop() {
    eMotor.stopMotor();
  }
}
