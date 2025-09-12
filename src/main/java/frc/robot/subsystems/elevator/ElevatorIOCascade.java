package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorIOCascade extends ElevatorIO {

  // Helper variables
  private int lastReportedElevatorLevel = 1;
  private int lastReportedPivotPosition = 1;
  private double elevatorSpeed = 0.50;

  // Encoder value target.
  private double lastReportedEncoderValue = 0.000;

  // Motor, encoder, and config objects
  private SparkMax elevatorMotor = new SparkMax(Constants.eMotorID, MotorType.kBrushless);
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private EncoderConfig elevatorEncoderConfig = new EncoderConfig();
  private SparkMax pivotMotor = new SparkMax(Constants.pivotID, MotorType.kBrushless);
  private SparkMax intakeMotor = new SparkMax(Constants.intakeID, MotorType.kBrushless);
  private SparkBaseConfig elevatorMotorConfig = new SparkMaxConfig();
  private SparkBaseConfig pivotMotorConfig = new SparkMaxConfig();

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
    elevatorMotorConfig.inverted(true);
    pivotMotorConfig.inverted(false);
    elevatorMotorConfig.openLoopRampRate(0.2);
    elevatorMotorConfig.closedLoopRampRate(0.2);

    pivotMotorConfig.idleMode(IdleMode.kBrake);
    elevatorMotorConfig.idleMode(IdleMode.kBrake);

    // Apply encoder conversion rate
    elevatorEncoderConfig.positionConversionFactor(Constants.elevatorPositionConversionFactor);

    // Apply configuration
    elevatorMotor.configure(
        elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotMotor.configure(
        pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * This method will return the "level" corresponding to whatever sensor is active. If no sensor is
   * detecting, it will return -1
   */
  @Override
  public int getExactLevel() {
    if (bottomSwitch.get()) return 1;
    if (elevatorEncoder.getPosition() <= Constants.level2 + Constants.encoderMarginOfError
        && elevatorEncoder.getPosition() >= Constants.level2 - Constants.encoderMarginOfError)
      return 2;
    if (topSwitch.get()) return 3;
    return -1;
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
  // @Override
  public void oldSetLevel(double speed, int target) {
    speed = Math.abs(speed);

    if (target > 3) target = 3;
    if (target < 1) target = 1;

    if ((getLevel() == 1 && target == 2 && getExactLevel() != 2)
        || ((getLevel() == 1 || getLevel() == 2) && target == 3 && getExactLevel() != 3))
      elevatorMotor.set(speed);
    else if ((getLevel() == 3 && target == 2 && getExactLevel() != 2)
        || ((getLevel() == 3 || getLevel() == 2) && target == 1 && getExactLevel() != 1))
      elevatorMotor.set(-speed);
    else elevatorMotor.set(0);
  }

  // hiya there!! cleaned up some logic + made it look spiffy -L

  // Set a new encoder target (only set once in a command.)
  @Override
  public void reportEncoderValue() {
    lastReportedEncoderValue = elevatorEncoder.getPosition();
  }

  /**
   * Moves the elevator to the specified "level"
   *
   * @param targetPosition desired level to set the elevator to (range from 1 to 3)
   */
  @Override
  public void setLevel(int targetPosition) {
    // Stop the motor and do nothing if the movement is invalid.
    if (targetPosition < 1 || targetPosition > 3) {
      elevatorMotor.stopMotor();
      return;
    }

    // Check to see if the movement requested is valid. If not, stop the motor and return.
    boolean validMovement =
        ((targetPosition == 2)
                && (getExactLevel() != 2)
                && (lastReportedElevatorLevel != 2)) // to level 2
            || ((targetPosition == 3)
                && (getExactLevel() != 3)
                && (lastReportedElevatorLevel != 3)) // to level 3
            || ((targetPosition == 1)
                && (getExactLevel() != 1)
                && (lastReportedElevatorLevel != 1)); // to level 1

    if (!validMovement) {
      elevatorMotor.stopMotor();
      return;
    }

    // Determine speed direction.
    double input =
        Math.min(
            Math.max(
                0,
                (Math.PI / 2)
                    - ((elevatorEncoder.getPosition() - lastReportedEncoderValue)
                        / (Constants.encoderSetpoints[targetPosition - 1]
                            - lastReportedEncoderValue)
                        * (Math.PI / 2))),
            (Math.PI / 2));
    double currentAlpha =
        Math.max(.2, Math.sin(input)); // Change the decimal to change the lowest possible speed.
    double speed = currentAlpha;
    if ((targetPosition <= getExactLevel()) || (targetPosition <= lastReportedElevatorLevel))
      speed = -speed;

    // Set the speed.
    elevatorMotor.set(speed);
  }

  /**
   * Pivots the intake up or down
   *
   * @param speed The speed to run the motor at
   */
  @Override
  public void pivot(double speed) {
    // Prevent pivot movement entirely if we are at the bottom stage. Also stop moving.
    if (getExactLevel() == 1) {
      pivotMotor.stopMotor();
      return;
    }

    // If it does not actively detect the desired sensor, and the last reported pivot sensor is not
    // the desired sensor, enable movement to that position.
    // Logs the last reported position as the pivot may get jostled around mid-match, possibly
    // resulting the pivot moving into an irregular position.
    boolean canMoveForward = (!this.getSwitch(true) && (lastReportedPivotPosition != 2));
    boolean canMoveBackward = (!this.getSwitch(false) && (lastReportedPivotPosition != 1));

    if (canMoveForward && (speed > 0)) pivotMotor.set(speed);
    else if (canMoveBackward && (speed < 0)) pivotMotor.set(speed);
    else pivotMotor.stopMotor();
  }

  @Override
  public void resetPivot(double speed) {
    speed = Math.abs(speed);

    if (getExactLevel() != 2) pivotMotor.set(speed);
    else pivotMotor.stopMotor();
  }

  @Override
  public void pivotPos(double speed, int target) {
    speed = Math.abs(speed);

    if (target > 2) target = 2;
    if (target < 1) target = 1;

    if (!this.getSwitch(true) && target == 2) pivotMotor.set(speed);
    else if (!this.getSwitch(false) && target == 1) pivotMotor.set(-speed);
    else pivotMotor.stopMotor();
  }

  /**
   * Run the intake motor
   *
   * @param speed The speed to run the motor at
   */
  @Override
  public void activeIntake(double speed) {
    intakeMotor.set(speed);
  }

  /** Stop the elevator */
  @Override
  public void stop() {
    elevatorMotor.stopMotor();
  }

  /** Stop the pivot motor */
  @Override
  public void stopPivot() {
    pivotMotor.stopMotor();
  }

  /** Stop the intake motor */
  @Override
  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  // Periodic update functions.
  // Ran in the "periodicUpdates" function, updated every 20ms.

  /** Returns the last detected "level" of the elevator */
  @Override
  public int getLevel() {
    if (bottomSwitch.get()) {
      lastReportedElevatorLevel = 1;
      elevatorEncoder.setPosition(
          Constants.encoderSetpoints[
              0]); // p.s. it's at 0 because java tables start indexing at 0 :p
    }
    if (topSwitch.get()) {
      lastReportedElevatorLevel = 3;
      elevatorEncoder.setPosition(Constants.encoderSetpoints[2]); // same deal, 3 is 2.
    }
    if (elevatorEncoder.getPosition()
            <= Constants.encoderSetpoints[1] + Constants.encoderMarginOfError
        && elevatorEncoder.getPosition()
            >= Constants.encoderSetpoints[1] - Constants.encoderMarginOfError) {
      lastReportedElevatorLevel = 2;
    }
    return lastReportedElevatorLevel;
  }

  @Override
  public int getPivotPos() {
    // Update PivotPosition while we're at it.
    if (this.getSwitch(false)) lastReportedPivotPosition = 1;
    else if (this.getSwitch(true)) lastReportedPivotPosition = 2;
    return lastReportedPivotPosition;
  }

  /** Updates the dashboard information */
  @Override
  public void periodicUpdates() {
    SmartDashboard.putNumber("Elevator Level", getExactLevel());
    // SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getPosition());

    SmartDashboard.putNumber("Encoder Startpoint", lastReportedEncoderValue);
    SmartDashboard.putNumber("Elev. Encoder Value", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Last Reported Level", lastReportedElevatorLevel);

    SmartDashboard.putBoolean("Bottom switch", bottomSwitch.get());
    SmartDashboard.putBoolean("Top switch", topSwitch.get());
    SmartDashboard.putBoolean("Forward pivot", !forwardPivot.get());
    SmartDashboard.putBoolean("Reverse pivot", !reversePivot.get());

    // if (bottomSwitch.get()) elevatorEncoder.setPosition(Constants.level1);
    // if (topSwitch.get()) elevatorEncoder.setPosition(Constants.level2);

    getLevel();
    getPivotPos();
  }
}
