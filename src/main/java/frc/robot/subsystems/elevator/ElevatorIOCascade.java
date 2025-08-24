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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorIOCascade extends ElevatorIO {

  // Helper variables
  private int levelTracker = 1;
  private int pivotTracker = 1;
  private double elevatorSpeed = 0.45;

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

  /** Apply motor configuration settings */
  @Override
  public void configureMotors() {
    // Set Inversions & Ramp Rates
    eMConfig.inverted(true);
    pivotConfig.inverted(true);
    eMConfig.openLoopRampRate(0.2);
    eMConfig.closedLoopRampRate(0.2);

    pivotConfig.idleMode(IdleMode.kBrake);
    eMConfig.idleMode(IdleMode.kBrake);

    // Apply configuration
    eMotor.configure(eMConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * This method will return the "level" corresponding to whatever sensor is active. If no sensor is
   * detecting, it will return -1
   */
  @Override
  public int getExactLevel() {
    if (bottomSwitch.get()) return 1;
    // if (eEncoder.getPosition() <= Constants.level2 + 1
    //     && eEncoder.getPosition() >= Constants.level2 - 1) return 2;
    if (topSwitch.get()) return 3;
    return -1;
  }

  /** Returns the last detected "level" of the elevator */
  @Override
  public int getLevel() {
    if (bottomSwitch.get()) levelTracker = 1;
    if (topSwitch.get()) levelTracker = 3;
    // if (eEncoder.getPosition() <= Constants.level2 + 1
    //     && eEncoder.getPosition() >= Constants.level2 - 1) levelTracker = 2;
    return levelTracker;
  }

  @Override
  public int getPivotPos() {
    if (this.getSwitch(false)) pivotTracker = 1;
    else if (this.getSwitch(true)) pivotTracker = 2;
    return pivotTracker;
  }

  @Override
  public int getExactPivotPos() {
    if (this.getSwitch(false)) return 1;
    if (this.getSwitch(true)) return 2;
    return -1;
  }

  /** Returns the speed value of the elevator */
  @Override
  public double getSpeed() {
    return elevatorSpeed;
  }

  @Override
  public boolean getSwitch(boolean forward) {
    if (forward) return !forwardPivot.get();
    else return !reversePivot.get();
  }

  /**
   * Moves the elevator to the specified "level"
   *
   * @param speed The speed to run the motor at
   * @param level The desired "level," 1 or 2
   */
  @Override
  public void setLevel(double speed, int target) {
    speed = Math.abs(speed);

    if (target > 3) target = 3;
    if (target < 1) target = 1;

    if ((getLevel() == 1 && target == 2 && getExactLevel() != 2)
        || ((getLevel() == 1 || getLevel() == 2) && target == 3 && getExactLevel() != 3))
      eMotor.set(speed);
    else if ((getLevel() == 3 && target == 2 && getExactLevel() != 2)
        || ((getLevel() == 3 || getLevel() == 2) && target == 1 && getExactLevel() != 1))
      eMotor.set(-speed);
    else eMotor.set(0);
  }

  /**
   * Pivots the intake up or down
   *
   * @param speed The speed to run the motor at
   */
  @Override
  public void pivot(double speed) {
    // if not reverse pivot and speed is positive pivot backward
    if (this.getSwitch(false) && (speed > 0)) pivot.set(speed);
    else if (this.getSwitch(true) && (speed < 0)) pivot.set(speed);
    else pivot.set(0);
  }

  @Override
  public void pivotPos(double speed, int target) {
    speed = Math.abs(speed);

    if (target > 2) target = 2;
    if (target < 1) target = 1;

    if (pivotTracker == 1 && !this.getSwitch(true) && target == 2) pivot.set(-speed);
    else if (pivotTracker == 2 && !this.getSwitch(false) && target == 1) pivot.set(speed);
    else pivot.set(0);
  }

  /**
   * Run the intake motor
   *
   * @param speed The speed to run the motor at
   */
  @Override
  public void activeIntake(double speed) {
    intake.set(speed);
  }

  /** Stop the elevator */
  @Override
  public void stop() {
    eMotor.stopMotor();
  }

  /** Stop the pivot motor */
  @Override
  public void stopPivot() {
    pivot.stopMotor();
  }

  /** Stop the intake motor */
  @Override
  public void stopIntake() {
    intake.stopMotor();
  }

  /** Updates the dashboard information */
  @Override
  public void periodicUpdates() {
    SmartDashboard.putNumber("Elevator Level", getExactLevel());
    SmartDashboard.putNumber("Elevator Encoder", eEncoder.getPosition());

    SmartDashboard.putNumber("E Encoder", eEncoder.getPosition());

    SmartDashboard.putBoolean("Bottom switch", bottomSwitch.get());
    SmartDashboard.putBoolean("Top switch", topSwitch.get());
    SmartDashboard.putBoolean("Forward pivot", !forwardPivot.get());
    SmartDashboard.putBoolean("Reverse pivot", !reversePivot.get());

    // if (bottomSwitch.get()) eEncoder.setPosition(Constants.level1);
    // if (topSwitch.get()) eEncoder.setPosition(Constants.level2);

    getLevel();
    getPivotPos();
  }
}
