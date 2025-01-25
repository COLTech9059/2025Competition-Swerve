package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class ElevatorIOSpark extends ElevatorIO {

  // Motor, encoder, and config objects
  private SparkMax eMotor = new SparkMax(Constants.eMotorID, MotorType.kBrushless);
  private RelativeEncoder eEncoder = eMotor.getEncoder();
  private SparkMax eMotor2 = new SparkMax(Constants.eMotor2ID, MotorType.kBrushless);
  private RelativeEncoder eEncoder2 = eMotor2.getEncoder();
  private SparkMax intake = new SparkMax(Constants.intakeID, MotorType.kBrushless);
  private SparkMax algae = new SparkMax(Constants.algaeID, MotorType.kBrushless);
  private SparkBaseConfig eMConfig;
  private SparkBaseConfig eM2Config;

  // Digital input (limit switch) objects
  private DigitalInput l1Switch = new DigitalInput(Constants.l1SwitchID);
  private DigitalInput l2Switch = new DigitalInput(Constants.l2SwitchID);
  private DigitalInput l3Switch = new DigitalInput(Constants.l3SwitchID);

  // Apply all necessary motor configs
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

  // Get the average between the two encoders
  private double getEncoderAverage() {
    return (eEncoder.getPosition() + eEncoder2.getPosition()) / 2;
  }

  // Set the encoders to a specific position
  private void setEncoders(double pos) {
    eEncoder.setPosition(pos);
    eEncoder2.setPosition(pos);
  }

  // Move the elevator to the given level at the given speed
  @Override
  public void setLevel(double speed, int level) {
    speed = Math.abs(speed);
    if (getLevel() > level) eMotor.set(-speed);
    if (getLevel() < level) eMotor.set(speed);
    if (getLevel() == level) eMotor.stopMotor();

    if (level > 3) level = 3;
    if (level < 1) level = 1;
  }

  // TEMPORARY; simply for testing the motor
  @Override
  public void runMotor(double speed){
    int level = getLevel();
    boolean yesTerminate = ((level == 1 && speed < 0) || ((getEncoderAverage() >= Constants.level2 || level == 2) && speed > 0));
    if (yesTerminate) {eMotor.set(0); return;}
    eMotor.set(speed);
  }

  // Returns the current level of the elevator (0 means none)
  @Override
  public int getLevel() {
    if (l1Switch.get()) return 1;
    if (l2Switch.get()) return 2;
    if (l3Switch.get()) return 3;
    return 0;
  }

  // Stops the elevator motors
  @Override
  public void stop() {
    eMotor.stopMotor();
  }

  // Runs the intake motor without stopping it
  @Override
  public void activeIntake(double speed) {
    intake.set(speed);
  }

  // Runs the intake at the set speed for a specific amount of time
  @Override
  public void timedIntake(double speed, double time) {
    Timer timer = new Timer();
    timer.reset();
    timer.start();

    intake.set(speed);
    if (timer.get() >= time) intake.stopMotor();
  }

  // Runs the intake until a sensor is triggered
  @Override
  public void sensorIntake(double speed) {} /* This will only be used if a sensor is placed in the intake */

  // Stops the intake motor
  @Override
  public void stopIntake() {
    intake.stopMotor();
  }

  // Set the algae motor to the given speed
  @Override
  public void algaeIntake(double speed) {
    algae.set(speed);
  }

  // Set the algae motor to the given speed for a given amount of time
  @Override
  public void timedAlgae(double speed, double time) {
    Timer timer = new Timer();
    timer.reset();
    timer.start();

    algae.set(speed);
    if (timer.get() >= time) algae.stopMotor();
  }

  // Stops the algae motor
  @Override
  public void stopAlgae() {
    algae.stopMotor();
  }

  // Updates encoder values according to elevator level
  @Override
  public void periodic() {
    if (getLevel() == 1) setEncoders(Constants.level1);
    if (getLevel() == 2) setEncoders(Constants.level2);
    if (getLevel() == 3) setEncoders(Constants.level3);
  }
}
