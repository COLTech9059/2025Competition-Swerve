package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class ElevatorIOSparkTest extends ElevatorIO {

  // Define motor objects
  private SparkMax eMotor = new SparkMax(Constants.eMotorID, MotorType.kBrushless);
  private RelativeEncoder eEncoder = eMotor.getEncoder();
  private SparkMaxConfig eConfig = new SparkMaxConfig();

  @Override
  public void configureMotors() {
    eConfig.inverted(true);
    eConfig.openLoopRampRate(0.2);
    eConfig.closedLoopRampRate(0.2);

    eMotor.configure(eConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void runMotor(double speed) {
    DriverStation.reportWarning("Running Elevator Motor.", false);
    eMotor.set(speed);
  }

  @Override
  public void setVoltage(double volts) {
    eMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    DriverStation.reportWarning("Stopping Elevator Motor.", false);
    eMotor.stopMotor();
  }
}
