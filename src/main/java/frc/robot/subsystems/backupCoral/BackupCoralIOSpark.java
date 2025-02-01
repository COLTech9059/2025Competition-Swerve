package frc.robot.subsystems.backupCoral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class BackupCoralIOSpark extends BackupCoralIO {

  private Timer timer = new Timer();

  private SparkMax pivotMotor = new SparkMax(Constants.pivotMotorID, MotorType.kBrushless);
  private SparkMaxConfig pivotConfig = new SparkMaxConfig();
  private SparkMax intakeMotor = new SparkMax(Constants.backupIntakeID, MotorType.kBrushless);
  private SparkMaxConfig intakeConfig = new SparkMaxConfig();

  private DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  private DoubleSolenoid piston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);

  private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  @Override
  public void configure() {
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    compressor.enableDigital();
  }

  @Override
  public void timedPivot(double speed, double time) {
    timer.reset();
    timer.start();

    pivotMotor.set(speed);
    if (timer.get() >= time) pivotMotor.stopMotor();
  }

  /**This mehtod will only be employed if a sensor is used for the pivot */
  @Override
  public void sensorPivot(double speed) {}

  @Override
  public void timedIntake(double speed, double time) {
    timer.reset();
    timer.start();

    intakeMotor.set(speed);
    if (timer.get() >= time) intakeMotor.stopMotor();
  }

  @Override
  public void levelSwap() {
    piston.toggle();
    piston2.toggle();
  }
}
