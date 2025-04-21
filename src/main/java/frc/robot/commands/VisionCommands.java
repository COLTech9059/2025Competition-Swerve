package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.Cameras.centerType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import java.util.function.DoubleSupplier;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionCommands {

  // Helpers
  private static DoubleSupplier[] speedsToAlign(
      PhotonTrackedTarget target, Transform3d center, Transform3d camPoseToRobotCenter) {
    // Important Values
    double yaw = target.getYaw(); // Angle to flat (+ right, - left)

    // bit of a misnomer but the function below returns the best estimation of camera pose to
    // target.
    // X -> forward distance, Y -> (Left-Positive) horizontal distance Z->, height
    Transform3d tOffset = target.getBestCameraToTarget();

    // calc offsets (MAGNITUDES!) also just y2-y1
    // might be y1-y2 though or if it is accurate altogther
    double centerOffset = (camPoseToRobotCenter.getY() - center.getY());

    // Distance calculation (using absolute value because idk if getx will give a negative)
    double dist = Math.abs(tOffset.getX()) + Math.abs(centerOffset * Math.sin(Math.toRadians(yaw)));
    // captured AprilTag (where the 1 is)

    // Horizontal offset
    double horizontal =
        tOffset.getY()
            + (centerOffset
                * Math.cos(
                    Math.toRadians(
                        yaw))); // TODO: needs horizontal offset compared to the camera that
    // captured AprilTag (where the 1 is)

    DoubleSupplier xSupply = () -> (.2 * dist);
    DoubleSupplier ySupply = () -> (.5 * horizontal);
    DoubleSupplier omegaSupply = () -> (yaw * .4);

    // Cutoffs
    // forced to implement these because we would burn carpets and thats not very cash money
    if (dist < .25) xSupply = () -> (0);
    if (Math.abs(horizontal) < .1) ySupply = () -> (0);
    if (Math.abs(yaw) < 1) omegaSupply = () -> (0);

    return new DoubleSupplier[] {xSupply, ySupply, omegaSupply};
  }

  // Main commands (ones that actually return Commands)

  public static Command alignToAprilTag(Drive driveSub, Vision visionSub, centerType centerBias) {
    // get the id of the camera we are using
    int camToUse = visionSub.determineBestCamera();

    SmartDashboard.putNumber("bestCamera", camToUse);
    // return nothing if we don't have a camera
    if (camToUse == -1) return Commands.none();

    // get camera + target data
    VisionIOInputsAutoLogged cam = visionSub.getInputCamera(camToUse);
    PhotonTrackedTarget target = visionSub.getBestTarget(camToUse);

    // find the center we want to align about.
    Transform3d center =
        new Transform3d(
            cam.camRobotOffset.getX(),
            0.0,
            cam.camRobotOffset.getZ(),
            new Rotation3d()); // Center of robot but at the same height and plane as the cameras
    switch (centerBias) {
      case RIGHT:
        center = Constants.Cameras.robotToCamera1;
        break;
      case LEFT:
        center =
            Constants.Cameras
                .robotToCamera0; // taking constants because we don't know what camera the program
        // will use
        break;
      case CENTER:
      default:
    }

    // Post the fiducial ID of the tag (if any) to smartDashboard for debugging (if there is no
    // presence then something is wrong.)
    SmartDashboard.putNumber("fiducialID", target.getFiducialId());

    // Checks to see if the tag is on a whitelist. Does nothing if it is not on the whitelist.
    boolean valid = false;
    for (int tagId : Constants.Cameras.tagWhitelist) {
      if (target.getFiducialId() == tagId) valid = true;
    }

    SmartDashboard.putBoolean("isValid", valid);

    if (!valid) return Commands.none();

    // Calculate speed values to input into RobotRelativeDrive
    DoubleSupplier[] speedValues = speedsToAlign(target, center, cam.camRobotOffset);

    // steal the robot relative drive command
    return DriveCommands.robotRelativeDrive(
        driveSub, speedValues[0], speedValues[1], speedValues[2]);
  }

  // Simple function to just
  public static Command OrientToAprilTagPlane(Drive drive, Vision vision) {
    // get the id of the camera we are using
    DriverStation.reportWarning("Determining best camera!", false);
    int camToUse = vision.determineBestCamera();
    DriverStation.reportWarning("Determined best camera!", false);
    SmartDashboard.putNumber("bestCamera", camToUse);
    // return nothing if we don't have a camera
    if (camToUse == -1) return Commands.none();

    // get camera + target data
    VisionIOInputsAutoLogged cam = vision.getInputCamera(camToUse);

    TargetObservation latestObservation = cam.latestTargetObservation;

    // Yaw
    double yaw =
        Math.abs(latestObservation.tx().getRadians()) > Math.PI / 24
            ? latestObservation.tx().getRadians()
            : 0;

    // I'm lazy so here's the definition of a zero for a double supplier.
    DoubleSupplier Zero = () -> (0);

    return DriveCommands.robotRelativeDrive(drive, Zero, Zero, () -> (yaw));
  }
}
