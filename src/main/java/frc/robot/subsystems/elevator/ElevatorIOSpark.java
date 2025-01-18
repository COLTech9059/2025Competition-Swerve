package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorIOSpark extends ElevatorIO{

  private SparkMax eMotor = new SparkMax(Constants.eMotorID, MotorType.kBrushless);
  private RelativeEncoder eEncoder = eMotor.getEncoder();
  private SparkMax eMotor2 = new SparkMax(Constants.eMotor2ID, MotorType.kBrushless);
  private RelativeEncoder eEncoder2 = eMotor2.getEncoder();
  private SparkMax intake = new SparkMax(Constants.intakeID, MotorType.kBrushless);
  private SparkMax algae = new SparkMax(Constants.algaeID, MotorType.kBrushless);
  private SparkBaseConfig eMConfig;
  private SparkBaseConfig eM2Config;

  private DigitalInput l1Switch = new DigitalInput(Constants.l1SwitchID);
  private DigitalInput l2Switch = new DigitalInput(Constants.l2SwitchID);
  private DigitalInput l3Switch = new DigitalInput(Constants.l3SwitchID);

  @Override
  public void configureMotors() {
    // eM2 output will now mirror eM outpput
    eM2Config.follow(eMotor, false);

    // Set Inversions & Ramp Rates
    eMConfig.inverted(false);
    eMConfig.openLoopRampRate(0.2);
    eMConfig.closedLoopRampRate(0.2);
    eM2Config.openLoopRampRate(0.2);
    eM2Config.closedLoopRampRate(0.2);

    // Apply configuration
    eMotor2.configure(eM2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    eMotor.configure(eM2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private double getEncoderAverage() {
    return (eEncoder.getPosition() + eEncoder2.getPosition())/2;
  }

 private void setEncoders(double pos) {
    eEncoder.setPosition(pos);
    eEncoder2.setPosition(pos);
  }

  @Override
  public void setLevel(double speed, int level) {
    speed = Math.abs(speed);
    if (getLevel() > level) eMotor.set(speed);
    if (getLevel() < level) eMotor.set(speed);
    if (getLevel() == level) eMotor.stopMotor();

    if (level > 3) level = 3;
  }

  @Override
  public int getLevel() {
    int level = 0;
    if (l1Switch.get()) level = 1;
    if (l2Switch.get()) level = 2;
    if (l3Switch.get()) level = 3;

    return level;
  }

  @Override
  public void stop() {
    eMotor.stopMotor();
  }

  @Override
  public void activeIntake(double speed) {
    intake.set(speed);
  }

  @Override
  public void timedIntake(double speed, double time) {
    Timer timer = new Timer();
    timer.reset(); timer.start();

    intake.set(speed);
    if (timer.get() >= time) intake.stopMotor();
  }

  @Override
  public void sensorIntake(double speed) {} /* This will only be used if a sensor is placed in the intake */

  @Override
  public void stopIntake() {
    intake.stopMotor();
  }

  @Override
  public void algaeIntake(double speed) {
    algae.set(speed);
  }

  @Override
  public void timedAlgae(double speed, double time) {
    Timer timer = new Timer();
    timer.reset(); timer.start();

    algae.set(speed);
    if (timer.get() >= time) algae.stopMotor();
  }

  @Override
  public void stopAlgae() {
    algae.stopMotor();
  }

  @Override
  public void periodic() {
    if (getLevel() == 1) setEncoders(Constants.level1); 
    if (getLevel() == 2) setEncoders(Constants.level2); 
    if (getLevel() == 3) setEncoders(Constants.level3); 
 }

}
