package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Hardware class for Elevator which uses SparkMaxes
 *
 * @author DevAspen & SomnolentStone
 */
public class ElevatorIOSpark extends ElevatorIO {

  // Helper variables
  private int levelTracker = 0;
  private double elevatorSpeed = 0;

  // Elevator speed controllers
  private Timer speedTimer = new Timer();
  private double speedDeadband = .1;
  private double period = 2; // Because it being a sin function, Math.PI is the base period that would actually be used. A period value of 2 would make half of the period 2 seconds.

  // Motor, encoder, and config objects
  private SparkMax eMotor = new SparkMax(Constants.eMotorID, MotorType.kBrushless);
  private RelativeEncoder eEncoder = eMotor.getEncoder();
  private SparkMax pivot = new SparkMax(Constants.pivotID, MotorType.kBrushless);
  private SparkMax intake = new SparkMax(Constants.intakeID, MotorType.kBrushless);
  private SparkBaseConfig eMConfig = new SparkMaxConfig();
  private SparkBaseConfig pivotConfig = new SparkMaxConfig();

  // Elevator limit switches
  private DigitalInput bottomSwitch = new DigitalInput(Constants.level0ID);
  private DigitalInput topSwitch = new DigitalInput(Constants.level1ID);

  // Pivot limit switches
  private DigitalInput forwardPivot = new DigitalInput(Constants.pivotForwardSwitch);
  private DigitalInput reversePivot = new DigitalInput(Constants.pivotReverseSwitch);

  // Apply all necessary motor configs
  @Override
  public void configureMotors() {
    // Set Inversions & Ramp Rates
    eMConfig.inverted(false);
    // eM2Config.inverted(false);
    eMConfig.openLoopRampRate(0.2);
    eMConfig.closedLoopRampRate(0.2);

    pivotConfig.idleMode(IdleMode.kBrake);
    // eM2Config.openLoopRampRate(0.2);
    // eM2Config.closedLoopRampRate(0.2);

    // Apply configuration
    // eMotor2.configure(eM2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    eMotor.configure(eMConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Get the average between the two encoders
  private double getEncoderAverage() {
    return (eEncoder.getPosition());
  }

  // Set the encoders to a specific position
  private void setEncoders(double pos) {
    eEncoder.setPosition(pos);
    // eEncoder2.setPosition(pos);
  }

  @Override
  public void pivot(double speed) {
    pivot.set(speed);
  }

  @Override
  public void stopPivot() {
    pivot.stopMotor();
  }


  // Runs the elevator with dynamic speed.
  // Ramps up the speed to max, then reduces it back to zero.
  // Deadbands are available to guarantee the elevator always runs.
  public void runElevatorWithPeriod(double speed, int level){
    if (getExactLevel() == level) {
      speedTimer.stop();
      speedTimer.reset();

      eMotor.stopMotor();
      return;
    }
    
    // start the timer
    if (speedTimer.get() == 0) {
      speedTimer.start();
    }
    // Speed multiplied by the result of a sin function (but really it's half of a sin function.) Value from 0-1.

    double newSpeed = speed * Math.sin((speedTimer.get()/period) * Math.PI);
    if (newSpeed < speedDeadband) newSpeed = speedDeadband;
    eMotor.set(newSpeed);
  }
  // Move the elevator to the given level at the given speed
  @Override
  public void setLevel(double speed, int level) {
    speed = Math.abs(speed);
    if (getExactLevel() == level) {
      eMotor.stopMotor();
      // eMotor2.stopMotor();
    } else {

      if (getLevel() > level) {
        eMotor.set(speed);
      }

      if (getLevel() < level) {
        eMotor.set(-speed);
      }
    }

    if (level > 1) level = 1;
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

  // Returns the current level of the elevator
  @Override
  public int getLevel() {
    if (!bottomSwitch.get()) levelTracker = 0;
    if (!topSwitch.get()) levelTracker = 1;
    // if (l2Switch.get() && stage2Switch.get()) levelTracker = 2;
    // if (l3Switch.get()) levelTracker = 3;
    return levelTracker;
  }

  /**
   * This method will return the "level" corresponding to whatever sensor is active. If no sensor is
   * detecting, it will return a -1
   */
  @Override
  public int getExactLevel() {
    if (!bottomSwitch.get()) return 0;
    if (!topSwitch.get()) return 1;
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

  @Override
  public double getSpeed() {
    return elevatorSpeed;
  }

  // Increments the elevator speed
  @Override
  public void incrementSpeed(double value) {
    elevatorSpeed += Math.abs(value);
    if (elevatorSpeed > 1) elevatorSpeed = 1;
  }

  // Decrements the elevator speed
  @Override
  public void decrementSpeed(double value) {
    elevatorSpeed -= Math.abs(value);
    if (elevatorSpeed < -1) elevatorSpeed = -1;
  }

  @Override
  public boolean getSwitch(boolean forward) {
    if (forward) return forwardPivot.get();
    else return reversePivot.get();
  }

  // Updates encoder values according to elevator level
  @Override
  public void periodicUpdates() {
    SmartDashboard.putNumber("Elevator Level", getExactLevel());

    SmartDashboard.putBoolean("Bottom switch", !bottomSwitch.get());
    SmartDashboard.putBoolean("Top switch", !topSwitch.get());
    SmartDashboard.putBoolean("Forward pivot", forwardPivot.get());
    SmartDashboard.putBoolean("Reverse pivot", reversePivot.get());

    // if (getLevel() == 1) setEncoders(Constants.level1);
    // if (getLevel() == 2) setEncoders(Constants.level2);
    // if (getLevel() == 3) setEncoders(Constants.level3);
  }
}
