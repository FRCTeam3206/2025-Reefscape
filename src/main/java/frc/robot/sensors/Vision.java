package frc.robot.sensors;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionSimConstants;
import frc.robot.Robot;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  final VisionSystemSim visionSim;
  private Matrix<N3, N1> curStdDevs;
  private double yawToTag = 0;
  private double tagArea = 0;
  private int goalID = 7;
  private boolean targetVisible = false;

  public Vision(String cameraName, Transform3d robotToCamera, VisionSystemSim visionSim) {
    this.visionSim = visionSim;
    camera = new PhotonCamera(cameraName);
    photonEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kTagLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera);

    if (Robot.isSimulation()) {
      SimCameraProperties cameraProperties = new SimCameraProperties();
      cameraProperties.setCalibration(
          VisionSimConstants.kCameraWidth,
          VisionSimConstants.kCameraHeight,
          VisionSimConstants.kDiagonalFOV);
      cameraProperties.setCalibError(
          VisionSimConstants.kAvgDetectionNoisePixels,
          VisionSimConstants.kStdDevDetectionNoisePixels);
      cameraProperties.setFPS(VisionSimConstants.kImageCaptureFPS);
      cameraProperties.setAvgLatencyMs(VisionSimConstants.kAvgLatencyMs);
      cameraProperties.setLatencyStdDevMs(VisionSimConstants.kStdDevLatencyMs);

      final PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProperties);
      visionSim.addCamera(cameraSim, robotToCamera);
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : camera.getAllUnreadResults()) {
      visionEst = photonEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());

      if (Robot.isSimulation()) {
        visionEst.ifPresentOrElse(
            est ->
                getSimDebugField()
                    .getObject("VisionEstimation")
                    .setPose(est.estimatedPose.toPose2d()),
            () -> {
              getSimDebugField().getObject("VisionEstimation").setPoses();
            });
      }
    }
    return visionEst;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      // Don't trust this because it doesn't know anything.
      curStdDevs = VisionConstants.kUntrustworthyStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = VisionConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      PhotonTrackedTarget firstTarget = new PhotonTrackedTarget();
      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        if (numTags == 0) {
          firstTarget = tgt;
        }
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        // Don't trust this because it doesn't know anything.
        curStdDevs = VisionConstants.kUntrustworthyStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Don't trust if there is only one tag and either the ambiguity is greater than 0.2 or it
        // returns -1 (meaning it's invalid).
        if (numTags == 1
            && (firstTarget.getPoseAmbiguity() > 0.2 || firstTarget.getPoseAmbiguity() == -1)) {
          estStdDevs = VisionConstants.kUntrustworthyStdDevs;
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) estStdDevs = VisionConstants.kUntrustworthyStdDevs;
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }

  public void updateYawDistance(int newGoal) {
    goalID = newGoal;
    updateYawDistance();
  }

  public void updateYawDistance() {
    var results = camera.getAllUnreadResults();
    targetVisible = false;
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == goalID) {
            yawToTag = target.getYaw();
            tagArea = target.getArea();
            targetVisible = true;
          }
        }
      }
    }
  }

  public double getYawToTag() {
    return yawToTag;
  }

  public double getTagArea() {
    return tagArea;
  }

  public int getGoalID() {
    return goalID;
  }

  public boolean getTargetVisible() {
    return targetVisible;
  }
}
