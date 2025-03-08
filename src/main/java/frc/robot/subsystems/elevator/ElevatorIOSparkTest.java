package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Alternative hardware class for Elevator which uses SparkMaxes for a testbed setup
 *
 * @author DevAspen
 */
public class ElevatorIOSparkTest extends ElevatorIO {

  private int levelTracker = 0;
  private double elevatorSpeed = 0.35;

  // Define motor objects
  private final SparkMax eMotor = new SparkMax(Constants.eMotorID, MotorType.kBrushless);
  private final RelativeEncoder eEncoder = eMotor.getEncoder();
  private final SparkMaxConfig eConfig = new SparkMaxConfig();

  // Define limit switch objects
  private final DigitalInput l0Switch = new DigitalInput(0);
  private final DigitalInput stopSwitch = new DigitalInput(1);

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
  public void setLevel(double speed, int level) {
    speed = Math.abs(speed);
    if (getExactLevel() == level) {
      eMotor.stopMotor();
      return;
    }
    if (getLevel() < level) {
      eMotor.set(speed);
    } else {
      eMotor.set(-speed);
    }
  }

  @Override
  public void setVoltage(double volts) {
    eMotor.setVoltage(volts);
  }

  @Override
  public void runToSensor(double speed) {
    eMotor.set(speed);
    if (l0Switch.get()) eMotor.stopMotor();
  }

  @Override
  public boolean getSwitch() {
    return l0Switch.get();
  }

  @Override
  public void stop() {
    DriverStation.reportWarning("Stopping Elevator Motor.", false);
    eMotor.stopMotor();
  }

  @Override
  public int getLevel() {
    if (l0Switch.get()) levelTracker = 0;
    if (stopSwitch.get()) levelTracker = 2;
    return levelTracker;
  }

  @Override
  public int getExactLevel() {
    if (l0Switch.get()) return 0;
    if (stopSwitch.get()) return 2;
    return -1;
  }

  @Override
  public void periodicUpdates() {
    SmartDashboard.putBoolean("Bottom Switch", l0Switch.get());
    SmartDashboard.putBoolean("Top Switch", stopSwitch.get());

    if (l0Switch.get()) eEncoder.setPosition(Constants.level0);
    if (stopSwitch.get()) eEncoder.setPosition(Constants.level2);
  }
}
