// Copyright (c) 2024-2025 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DriveCommands {

  // Needed for Characterization routines, not normal robot operations
  // DO NOT ADJUST
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command fieldRelativeDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get the Linear Velocity & Omega from inputs
          Translation2d linearVelocity =
              getLinearVelocity(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = getOmega(omegaSupplier.getAsDouble());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Robot relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command robotRelativeDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get the Linear Velocity & Omega from inputs
          Translation2d linearVelocity =
              getLinearVelocity(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = getOmega(omegaSupplier.getAsDouble());

          // Run with straight-up velocities w.r.t. the robot!
          drive.runVelocity(
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec()));
        },
        drive);
  }

  /**
   * Calculates the DoubleSupplier values to be plugged into robotRelativeDrive so the robot can
   * line up with a target april tag
   *
   * @param target The target april tag
   * @param cOffset The offset of the camera from the center of the robot
   * @param direction the direction of the camera compared to the center of the robot; positive or
   *     negative 1 only.
   * @return An array of DoubleSuppliers. [0] is for x, [1] is for y, and [2] is for omega
   */
  // public static DoubleSupplier[] alignmentCalculation(PhotonTrackedTarget target, Transform3d
  // cOffset, double direction) {

  //   double yaw = target.getYaw(); // Angle to flat (+ right, - left)
  //   Transform3d tOffset = target.bestCameraToTarget; // Returns the distance to target (x),
  // horizontal offset (y), and vertical offset (z)
  //   double totalOffset = tOffset.getY() + cOffset.getX();
  //   double xOffset = tOffset.getX();
  //   DoubleSupplier xSupply = () -> 0;
  //   DoubleSupplier ySupply = () -> 0;
  //   DoubleSupplier omegaSupply = () -> 0;

  //   if (Math.abs(yaw) >= 10) {
  //     xSupply = () -> 0;
  //     ySupply = () -> 0;
  //     omegaSupply = () -> yaw * 0.4;
  //   } else if (Math.abs(totalOffset) >= 3 && Math.abs(xOffset) >= 3) {
  //     xSupply = () -> 0.1 * xOffset;
  //     ySupply = () -> 0.5 * totalOffset;
  //     omegaSupply = () -> 0;
  //   }

  //   return new DoubleSupplier[] {xSupply, ySupply, omegaSupply};
  // }

  public static DoubleSupplier[] alignmentCalculation(
      PhotonTrackedTarget target, Transform3d cOffset, double direction) {
    double yaw = target.getYaw(); // Angle to flat (+ right, - left)
    Transform3d tOffset =
        target.bestCameraToTarget; // Returns the distance to target (x), horizontal offset (y), and
    // vertical offset (z)
    double totalOffset = tOffset.getY() + cOffset.getX();
    double xOffset = tOffset.getX();
    DoubleSupplier xSupply = () -> 0;
    DoubleSupplier ySupply = () -> 0;
    DoubleSupplier omegaSupply = () -> 0;

    if (Math.abs(yaw) >= 10) {
      xSupply = () -> 0;
      ySupply = () -> 0;
      omegaSupply = () -> yaw * 0.4;
    } else if (Math.abs(totalOffset) >= 3 && Math.abs(xOffset) >= 3) {
      xSupply = () -> 0.1 * xOffset;
      ySupply = () -> 0.5 * totalOffset;
      omegaSupply = () -> 0;
    }

    return new DoubleSupplier[] {xSupply, ySupply, omegaSupply};
  }

  /**
   * Calculates the DoubleSupplier values to be plugged into robotRelativeDrive so the robot can
   * line up with a target april tag
   *
   * @param target The target april tag
   * @param center The center axis to align the april tag to.
   * @param direction the direction of the camera compared to the center of the robot; positive or
   *     negative 1 only.
   * @return An array of DoubleSuppliers. [0] is for x, [1] is for y, and [2] is for omega
   */
  public static DoubleSupplier[] alignmentCalculation(
      PhotonTrackedTarget target,
      Transform3d center,
      Transform3d camPoseToRobotCenter,
      Constants.Cameras.centerType centerType) {
    /* ok planning comments start now
      ok so considering offsets, the yaw isn't going to be perfectly accurate
      it's based on the center of the camera used, and changing the center offset will alter that angle
      however, we have the distance to the target and the offset of the target horizontally
      we can use trigonometric functions to get new angles, and that will help us become parallel to the target.

      Assuming getBestCameraToTarget returns an x value (distance) field-relative (states that it maps camera pose to target pose)...
      ...we can multiply the horizontal offset between the center by the sin of the angle to perpendicular (probably yaw)
      and then add it to the x value.

      now for the horizontal (y-axis)
      a bit tricky because if the center is oriented about the left camera but it detects a tag from the right camera, you would have to get the absolute value
      (as in, you can't simply just subtract one from the other.)
      so, we need the camera pose relative to the robot and the center to properly do this
      even with this, it'll take conditionals to determine what to do with the transformations
    */
    // Important Values
    double yaw = target.getYaw(); // Angle to flat (+ right, - left)
    Transform3d tOffset =
        target
            .getBestCameraToTarget(); // Returns the distance to target (x), horizontal offset, left
    // positive (y), and vertical offset (z)

    // calc offsets (PLEASE @DevAspen DO THIS i can't focus on this rn)

    // Distance calculation (using absolute value because idk if getx will give a negative)
    double dist =
        Math.abs(tOffset.getX())
            + Math.abs(
                1
                    * Math.sin(
                        Math.toRadians(
                            yaw))); // TODO: needs horizontal offset compared to the camera that
    // captured AprilTag (where the 1 is)

    // Horizontal offset
    double horizontal =
        tOffset.getY()
            + (1
                * Math.cos(
                    Math.toRadians(
                        yaw))); // TODO: needs horizontal offset compared to the camera that
    // captured AprilTag (where the 1 is)

    // double totalOffset = tOffset.getY() + center.getX();
    // double xOffset = tOffset.getX();
    DoubleSupplier xSupply = () -> (.2 * dist);
    DoubleSupplier ySupply = () -> (.5 * horizontal);
    DoubleSupplier omegaSupply = () -> (yaw * .4);

    // if (Math.abs(yaw) >= 10) {
    //   xSupply = () -> 0;
    //   ySupply = () -> 0;
    //   omegaSupply = () -> yaw * 0.4;
    // } else if (Math.abs(totalOffset) >= 3 && Math.abs(xOffset) >= 3) {
    //   xSupply = () -> 0.1 * xOffset;
    //   ySupply = () -> 0.5 * totalOffset;
    //   omegaSupply = () -> 0;
    // }

    return new DoubleSupplier[] {xSupply, ySupply, omegaSupply};
  }

  /**
   * Constructs and returns a robotRelativeDrive command with the required values to line up with
   * the target april tag
   *
   * @param drive The Drive subsystem
   * @param target The target april tag
   * @param cOffset The offset of the camera from the center of the robot
   * @param rOffset The offset of the robot from the april tag
   * @return The robotRelativeDrive Command
   */
  // public static Command targetAlignment(Drive drive, PhotonTrackedTarget target, Transform3d
  // cOffset, Transform2d rOffset) {
  //   DoubleSupplier[] list = alignmentCalculation(target, cOffset, rOffset);
  //   return robotRelativeDrive(drive, list[0], list[1], list[2]);
  // }

  /**
   * Constructs and returns a robotRelativeDrive command with the required values to line up with
   * the target april tag
   *
   * @param drive The Drive subsystem
   * @param vision The Vision subsystem
   * @author DevAspen, Revamped by SomnolentStone
   * @return A Blank Command (if no valid target) or the robotRelativeDrive Command
   */
  // public static Command targetAlignment(Drive drive, Vision vision) {
  //   int cameraToUse = vision.determineBestCamera();
  //   if (cameraToUse == -1) return Commands.none(); // Blank command, does nothing.

  //   VisionIOInputsAutoLogged camera = vision.getInputCamera(cameraToUse);
  //   PhotonTrackedTarget target = vision.getBestTarget(cameraToUse);

  //   // Check to see if target is on whitelist
  //   boolean valid = false;
  //   for (int tagId : Constants.Cameras.tagWhitelist) {
  //     if (target.getFiducialId() == tagId) valid = true;
  //   }
  //   if (!valid) return Commands.none();

  //   // Values to help center the target.
  //   double direction =
  //       camera.camRobotOffset.getX()
  //           / Math.abs(
  //               camera.camRobotOffset
  //                   .getX()); // Divide by the positive self to gain a value of 1 or -1.
  //   Transform3d camOffset =
  //       new Transform3d(
  //           camera.camRobotOffset.getX(),
  //           camera.camRobotOffset.getY() + 18.0,
  //           camera.camRobotOffset.getZ(),
  //           camera.camRobotOffset.getRotation());

  //   DoubleSupplier[] list = alignmentCalculation(target, camOffset, direction);
  //   return robotRelativeDrive(drive, list[0], list[1], list[2]);
  // }

  /**
   * Constructs and returns a robotRelativeDrive command with the required values to line up with
   * the target april tag
   *
   * @param drive The Drive subsystem
   * @param target The target april tag
   * @param cOffset The offset of the camera from the center of the robot
   * @param rOffset The offset of the robot from the april tag
   * @return The robotRelativeDrive Command
   */
  // public static Command targetAlignment(Drive drive, PhotonTrackedTarget target, Transform3d
  // cOffset, Transform2d rOffset) {
  //   DoubleSupplier[] list = alignmentCalculation(target, cOffset, rOffset);
  //   return robotRelativeDrive(drive, list[0], list[1], list[2]);
  // }

  /**
   * Constructs and returns a robotRelativeDrive command with the required values to line up with
   * the target april tag
   *
   * @param drive The Drive subsystem
   * @param vision The Vision subsystem
   * @param centerType The center type the method wants to use (right, left, center)
   * @author DevAspen, Revamped by SomnolentStone
   * @return A Blank Command (if no valid target) or the robotRelativeDrive Command
   */
  // public static Command targetAlignment(
  //     Drive drive, Vision vision, Constants.Cameras.centerType centerType) {
  //   int cameraToUse = vision.determineBestCamera();
  //   if (cameraToUse == -1) return Commands.none(); // Blank command, does nothing.

  //   VisionIOInputsAutoLogged camera = vision.getInputCamera(cameraToUse);
  //   PhotonTrackedTarget target = vision.getBestTarget(cameraToUse);

  //   // offset the center
  //   Transform3d center =
  //       new Transform3d(
  //           camera.camRobotOffset.getX(),
  //           0.0,
  //           camera.camRobotOffset.getZ(),
  //           new Rotation3d()); // Center of robot but at the same height and plane as the cameras
  //   switch (centerType) {
  //     case RIGHT:
  //       center = Constants.Cameras.robotToCamera1;
  //       break;
  //     case LEFT:
  //       center =
  //           Constants.Cameras
  //               .robotToCamera0; // taking constants because we don't know what camera the
  // program
  //       // will use
  //       break;
  //     case CENTER:
  //     default:
  //   }

  //   // Check to see if target is on whitelist
  //   boolean valid = false;
  //   for (int tagId : Constants.Cameras.tagWhitelist) {
  //     if (target.getFiducialId() == tagId) valid = true;
  //   }
  //   if (!valid) return Commands.none();

  //   // Values to help center the target.
  //   double direction =
  //       camera.camRobotOffset.getX()
  //           / Math.abs(
  //               camera.camRobotOffset
  //                   .getX()); // Divide by the positive self to gain a value of 1 or -1.
  //   // Transform3d camOffset = new Transform3d(camera.camRobotOffset.getX(),
  //   // camera.camRobotOffset.getY() + Units.inchesToMeters(18.0), camera.camRobotOffset.getZ(),
  //   // camera.camRobotOffset.getRotation());

  //   DoubleSupplier[] list = alignmentCalculation(target, center, direction);
  //   return robotRelativeDrive(drive, list[0], list[1], list[2]);
  // }

  // /**
  //  * Constructs and returns a robotRelativeDrive command with the required values to line up with
  //  * the target april tag
  //  *
  //  * @param drive The Drive subsystem
  //  * @param vision The Vision subsystem
  //  * @param rOffset A Transform2d representing the desired offset from the center of the robot to
  //  *     the tag
  //  * @author DevAspen, Revamped by SomnolentStone
  //  * @return A Blank Command (if no valid target) or the robotRelativeDrive Command
  //  */
  // public static Command targetAlignment(Drive drive, Vision vision, Transform2d rOffset) {
  //   Transform3d rOffset3d = new Transform3d(rOffset);

  //   int cameraToUse = vision.determineBestCamera();
  //   if (cameraToUse == -1) return Commands.none(); // Blank command, does nothing.

  //   VisionIOInputsAutoLogged camera = vision.getInputCamera(cameraToUse);
  //   PhotonTrackedTarget target = vision.getBestTarget(cameraToUse);

  //   // Check to see if target is on whitelist
  //   boolean valid = false;
  //   for (int tagId : Constants.Cameras.tagWhitelist) {
  //     if (target.getFiducialId() == tagId) valid = true;
  //   }
  //   if (!valid) return Commands.none();

  //   // Values to help center the target.
  //   double direction =
  //       camera.camRobotOffset.getX()
  //           / Math.abs(
  //               camera.camRobotOffset
  //                   .getX()); // Divide by the positive self to gain a value of 1 or -1.
  //   Transform3d camOffset =
  //       new Transform3d(
  //           camera.camRobotOffset.getX(),
  //           camera.camRobotOffset.getY() + Units.inchesToMeters(18.0),
  //           camera.camRobotOffset.getZ(),
  //           camera.camRobotOffset.getRotation());

  //   DoubleSupplier[] list = alignmentCalculation(target, camOffset.plus(rOffset3d), direction);
  //   return robotRelativeDrive(drive, list[0], list[1], list[2]);
  // }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command fieldRelativeDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            AutoConstants.kPPsteerPID.kP,
            AutoConstants.kPPsteerPID.kI,
            AutoConstants.kPPsteerPID.kD,
            new TrapezoidProfile.Constraints(
                DrivebaseConstants.kMaxAngularSpeed, DrivebaseConstants.kMaxAngularAccel));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocity(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Compute the new linear velocity from inputs, including applying deadbands and squaring for
   * smoothness. Also apply the linear velocity Slew Rate Limiter.
   */
  private static Translation2d getLinearVelocity(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), OperatorConstants.kDeadband);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    // NOTE: The x & y values range from -1 to +1, so their squares are as well
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Compute the new angular velocity from inputs, including applying deadbands and squaring for
   * smoothness. Also apply the angular Slew Rate Limiter.
   */
  private static double getOmega(double omega) {
    omega = MathUtil.applyDeadband(omega, OperatorConstants.kDeadband);
    return Math.copySign(omega * omega, omega);
  }

  /***************************************************************************/
  /** DRIVEBASE CHARACTERIZATION COMMANDS ********************************** */
  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * SwerveConstants.kDriveBaseRadiusMeters) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
