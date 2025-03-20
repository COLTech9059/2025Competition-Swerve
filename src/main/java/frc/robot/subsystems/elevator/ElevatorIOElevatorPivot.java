package frc.robot.subsystems.elevator;

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
public class ElevatorIOElevatorPivot extends ElevatorIO {

  // Motor, encoder, and config objects
  // private SparkMax eMotor = new SparkMax(Constants.eMotorID, MotorType.kBrushless);
  // private RelativeEncoder eEncoder = eMotor.getEncoder();
  private SparkMax pivot = new SparkMax(Constants.pivotID, MotorType.kBrushless);
  private SparkMax intake = new SparkMax(Constants.intakeID, MotorType.kBrushless);
  // private SparkBaseConfig eMConfig = new SparkMaxConfig();
  private SparkBaseConfig pivotConfig = new SparkMaxConfig();

  // Elevator limit switches
  // private DigitalInput bottomSwitch = new DigitalInput(Constants.level0ID);
  // private DigitalInput topSwitch = new DigitalInput(Constants.level1ID);

  // Pivot limit switches
  private DigitalInput forwardPivot = new DigitalInput(Constants.pivotForwardSwitch);
  private DigitalInput reversePivot = new DigitalInput(Constants.pivotReverseSwitch);

  // Apply all necessary motor configs
  @Override
  public void configureMotors() {
    // Set Inversions & Ramp Rates
    // eMConfig.inverted(false);
    // eM2Config.inverted(false);
    // eMConfig.openLoopRampRate(0.2);
    // eMConfig.closedLoopRampRate(0.2);

    pivotConfig.idleMode(IdleMode.kBrake);
    // eM2Config.openLoopRampRate(0.2);
    // eM2Config.closedLoopRampRate(0.2);

    // Apply configuration
    // eMotor2.configure(eM2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // eMotor.configure(eMConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Get the average between the two encoders
  // private double getEncoderAverage() {
  //   return (eEncoder.getPosition());
  // }

  // Set the encoders to a specific position
  private void setEncoders(double pos) {
    // eEncoder.setPosition(pos);
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
  public boolean getSwitch(boolean forward) {
    if (forward) return forwardPivot.get();
    else return reversePivot.get();
  }

  // Updates encoder values according to elevator level
  @Override
  public void periodicUpdates() {
    // SmartDashboard.putNumber("Elevator Level", getExactLevel());

    SmartDashboard.putBoolean("Forward pivot", forwardPivot.get());
    SmartDashboard.putBoolean("Reverse pivot", reversePivot.get());
  }
}
