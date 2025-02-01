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

  // Helper variable to track the relative position of the elevator when it isn't at a sensor
  private int levelTracker = 0;

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
  private DigitalInput l0Switch = new DigitalInput(Constants.level0ID);
  private DigitalInput l1Switch = new DigitalInput(Constants.level1ID);
  private DigitalInput l2Switch = new DigitalInput(Constants.level2ID);
  private DigitalInput stage2Switch = new DigitalInput(Constants.stage2ID);
  private DigitalInput l3Switch = new DigitalInput(Constants.level3ID);

  // Apply all necessary motor configs
  @Override
  public void configureMotors() {
    // Set Inversions & Ramp Rates
    eMConfig.inverted(false);
    eM2Config.inverted(false);
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
    if (getExactLevel() == level) {
      eMotor.stopMotor();
      eMotor2.stopMotor();
    } else {

      if (getLevel() > level) {
        
        if (!stage2Switch.get()) {
          eMotor.stopMotor();
          eMotor2.set(-speed);
        } else if (stage2Switch.get() && !l0Switch.get()) {
          eMotor.set(-speed);
          eMotor2.stopMotor();
        } else {
          eMotor.stopMotor();
          eMotor2.stopMotor();
        }
      }

      if (getLevel() < level) {

        if (l2Switch.get() && !l3Switch.get()) {
          eMotor.stopMotor();
          eMotor2.set(speed);
        } else if (!l2Switch.get()) {
          eMotor.set(speed);
          eMotor2.stopMotor();
        } else {
          eMotor.stopMotor();
          eMotor2.stopMotor();
        }
      }
    }

    if (level > 3) level = 3;
    if (level < 0) level = 0;
  }

  // TEMPORARY; simply for testing the motor
  @Override
  public void runMotor(double speed) {
    int level = getLevel();
    boolean yesTerminate =
        ((level == 1 && speed < 0)
            || ((getEncoderAverage() >= Constants.level2 || level == 2) && speed > 0));
    if (yesTerminate) {
      eMotor.set(0);
      return;
    }
    eMotor.set(speed);
  }

  // Returns the current level of the elevator (0 means none)
  @Override
  public int getLevel() {
    if (l0Switch.get()) levelTracker = 0;
    if (l1Switch.get()) levelTracker = 1;
    if (l2Switch.get() && stage2Switch.get()) levelTracker = 2;
    if (l3Switch.get()) levelTracker = 3;
    return levelTracker;
  }

  /** This method will return the "level" corresponding to whatever sensor is active. If no sensor is detecting, it will return a -1 */
  @Override
  public int getExactLevel() {
    if (l0Switch.get()) return 0;
    if (l1Switch.get()) return 1;
    if (l2Switch.get() && stage2Switch.get()) return 2;
    if (l3Switch.get()) return 3;
    return -1;
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
  public void sensorIntake(
      double speed) {} /* This will only be used if a sensor is placed in the intake */

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
