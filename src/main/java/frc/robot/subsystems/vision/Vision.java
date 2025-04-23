// Copyright (c) 2024-2025 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2024-2025 FRC 2486
// http://github.com/Coconuts2486-FRC
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

package frc.robot.subsystems.vision;

import static frc.robot.Constants.Cameras.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public VisionIOInputsAutoLogged getInputCamera(int camIndex) {
    return inputs[camIndex];
  }

  /**
   * Returns the camera ID of the most optimal camera to use. Uses the best target it checks (if
   * any)
   *
   * @author SomnolentStone
   * @return the index of the camera to use. -1 if no targets.
   */
  public int oldBestCamera() {
    // Store the comparison target and the camera ID.
    PhotonTrackedTarget lastTargetChecked = null;
    int camID = -1;

    // DEV NOTE: sorry for the setup; this is just in case we ever have a ton of cameras/one camera
    // is out of commission.
    // Loop through all targets. Compare with best.
    for (int i = 0; i < io.length; i++) {
      if (!inputs[i].connected) continue;
      PhotonTrackedTarget currentTarget = inputs[i].bestTarget;
      if (currentTarget == null) continue;
      if (i == 0) {
        lastTargetChecked = currentTarget;
        camID = i;
        continue;
      }
      if (lastTargetChecked != null) {
        // Simply compare target size on detection.
        double currentTargetArea = currentTarget.getArea();
        double comparisonArea = lastTargetChecked.getArea();

        if (currentTargetArea >= comparisonArea) {
          lastTargetChecked = currentTarget;
          camID = i;
        }
      }
    }

    return camID;
  }

  public int determineBestCamera() {
    // Store the comparison target and the camera ID.
    TargetObservation prevObservation = null;
    int camID = -1;
    // DriverStation.reportWarning("determining Camera!", false);
    // DEV NOTE: sorry for the setup; this is just in case we ever have a ton of cameras/one camera
    // is out of commission.
    // Loop through all targets. Compare with best.
    for (int i = 0; i < inputs.length; i++) {
      // DriverStation.reportWarning("Checking Camera " + i + "!", false);
      if (!inputs[i].connected) {
        continue;
      }
      // DriverStation.reportWarning("Camera " + i + " is enabled!", false);
      TargetObservation currentObservation = inputs[i].latestTargetObservation;
      if (currentObservation.equals(new TargetObservation(new Rotation2d(), new Rotation2d()))) {
        continue;
      }
      if (prevObservation == null) {
        prevObservation = currentObservation;
        camID = i;
        continue;
      }
      // Right-most is best.
      double currentYaw = currentObservation.tx().getRadians();
      double comparisonYaw = prevObservation.tx().getRadians();

      if (currentYaw >= comparisonYaw) {
        prevObservation = currentObservation;
        camID = i;
      }
    }

    return camID;
  }

  /**
   * Returns the best PhotonTrackedTarget in view of the specified camera
   *
   * @param cameraIndex The index of the camera to use
   */
  public PhotonTrackedTarget getBestTarget(int cameraIndex) {
    return inputs[cameraIndex].bestTarget;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = AprilTagConstants.aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > AprilTagConstants.aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > AprilTagConstants.aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
