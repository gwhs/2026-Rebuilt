package frc.robot.subsystems.objectDetection;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectionCam {

  private final PhotonCamera cam;

  private final Supplier<Pose2d> robotPose;
  private int counter;
  private final String ntKey;
  private final Transform3d robotToCam;
  private VisionSystemSim visionSim;
  private TargetModel simTargetModel;
  private Pose3d simTargetPose;
  private VisionTargetSim visionTarget;
  private SimCameraProperties cameraProp;
  private PhotonCameraSim cameraSim;
  private ArrayList<Pose3d> targetPoses;

  public ObjectDetectionCam(String name, Transform3d robotToCam, Supplier<Pose2d> robotPose) {

    cam = new PhotonCamera(name);
    counter = 0;
    this.robotPose = robotPose;
    this.robotToCam = robotToCam;
    targetPoses = new ArrayList<Pose3d>();

    ntKey = "/Object Detection/" + name + "/";
    if (RobotBase.isSimulation()) {
      this.initSim();
    }
  }

  public void initSim() {
    visionSim = new VisionSystemSim("main");
    simTargetModel = new TargetModel(0.2);

    simTargetPose =
        new Pose3d(15, 4, 0, new Rotation3d(0, 0, Math.PI)); // placeholder, change later
    visionTarget = new VisionTargetSim(simTargetPose, simTargetModel);
    visionSim.addVisionTargets(visionTarget);

    simTargetPose =
        new Pose3d(14, 6, 0, new Rotation3d(0, 0, Math.PI)); // placeholder, change later
    visionTarget = new VisionTargetSim(simTargetPose, simTargetModel);
    visionSim.addVisionTargets(visionTarget);
    // second piece maybe

    cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
    cameraProp.setFPS(100);
    cameraSim = new PhotonCameraSim(cam, cameraProp);
    cameraSim.enableDrawWireframe(true);
    visionSim.addCamera(cameraSim, robotToCam);
  }

  // 1. Get all results from the camera
  // 2. Check if the list is empty
  // 3. If not empty, loop through and getBestTarget
  // 4. Get target location in camera space
  // 5. Transform (target location in camera space) by (camera location in field space) to get
  // (target location in field space)
  // 6. Use three PID controllers x, y, and rotation to drive and rotate robot to target pose
  public void updateDetection() {

    if (RobotBase.isSimulation()) {
      visionSim.update(robotPose.get());
    }

    counter++;
    List<PhotonPipelineResult> results = cam.getAllUnreadResults();
    DogLog.log(ntKey + "Number of Results/", results.size());
    DogLog.log(ntKey + "counter", counter);

    Pose2d robotPose = this.robotPose.get();
    Pose3d robotPose3d = new Pose3d(robotPose);
    Pose3d cameraPose3d = robotPose3d.plus(robotToCam);
    DogLog.log(ntKey + "Camera Pose/", cameraPose3d);

    if (results.isEmpty()) {
      return;
    }

    for (PhotonPipelineResult result : results) {
      PhotonTrackedTarget targets = result.getBestTarget();
      if (targets == null) {
        continue;
      }

      // for(int i = 0; i < results.size(); i++) {
      //   var result = results.get(i);
      // }

      // Calculate the target's position in the field
      Pose3d targetPose;
      if (RobotBase.isSimulation()) {
        double targetYaw = -targets.getYaw();
        double targetPitch = targets.getPitch();
        Translation2d targetLocationToCamera = this.getCameraToTarget(targetYaw, targetPitch);
        Transform2d cameraToTargetTransform2d =
            new Transform2d(targetLocationToCamera, new Rotation2d());
        targetPose = new Pose3d(cameraPose3d.toPose2d().plus(cameraToTargetTransform2d.inverse()));
      } else {
        double targetYaw = -targets.getYaw();
        double targetPitch = targets.getPitch();
        Translation2d targetLocationToCamera = this.getCameraToTarget(targetYaw, targetPitch);
        Transform2d cameraToTargetTransform2d =
            new Transform2d(targetLocationToCamera, new Rotation2d());
        targetPose = new Pose3d(cameraPose3d.toPose2d().plus(cameraToTargetTransform2d.inverse()));
      }
      if (filterResults(targetPose)) {
        GamePieceTracker.addTarget(result.getTimestampSeconds(), targetPose.toPose2d());
        targetPoses.add(targetPose);
      }
      // add in game piece tracker then log in the same file (game piece tracker file maybe)
    }

    DogLog.log(ntKey + "Target Pose Array/", targetPoses.toArray(new Pose3d[0]));
    targetPoses.clear();
  }

  private Translation2d getCameraToTarget(double targetYaw, double targetPitch) {
    // Define the vector
    double x = 1.0 * Math.tan(Units.degreesToRadians(targetYaw));
    double y = 1.0 * Math.tan(Units.degreesToRadians(targetPitch));
    double z = 1.0;

    // DogLog.log(ntKey + "Target Pose Array/", );

    double norm = Math.sqrt(x * x + y * y + z * z);
    x /= norm;
    y /= norm;
    z /= norm;

    // Rotate the vector by the camera pitch
    Translation2d yzPrime =
        new Translation2d(y, z).rotateBy(new Rotation2d(robotToCam.getRotation().getY()));
    double yPrime = yzPrime.getX();

    // Solve for the intersection
    double angleToGoalRadians = Math.asin(yPrime);
    double diffHeight = robotToCam.getZ() - 0;
    double distance = diffHeight / Math.tan(angleToGoalRadians);

    return new Translation2d(distance, Rotation2d.fromDegrees(targetYaw));
  }

  public boolean filterResults(Pose3d detectedTargetPose) {

    // If vision’s detected target is below the ground/above tolerable height
    // double upperZBound = ObjectDetectionConstants.UPPER_Z_TOLERANCE;
    // double lowerZBound = ObjectDetectionConstants.LOWER_Z_TOLERANCE;
    // if (detectedTargetPose.getZ() > upperZBound
    //     || detectedTargetPose.getZ()
    //         < lowerZBound) { // change if we find out that z starts from camera height
    //   DogLog.log(ntKey + "Rejected Target Pose", detectedTargetPose);
    //   DogLog.log(ntKey + "Rejected Reason", "out of Z bounds", "Z: " +
    // detectedTargetPose.getZ());
    //   return false;
    // }

    // If vision's detected target pose is outside the field
    double upperXBound =
        ObjectDetectionConstants.MAX_X_VALUE + ObjectDetectionConstants.XY_TOLERANCE;
    double upperYBound =
        ObjectDetectionConstants.MAX_Y_VALUE + ObjectDetectionConstants.XY_TOLERANCE;
    double lowerXYBound = -(ObjectDetectionConstants.XY_TOLERANCE);
    if (detectedTargetPose.getX() < lowerXYBound || detectedTargetPose.getY() < lowerXYBound) {
      DogLog.log(ntKey + "Rejected Target Pose", detectedTargetPose);
      DogLog.log(ntKey + "Rejected Reason", "Y or X is less than 0");
      return false;
    }
    if (detectedTargetPose.getX() > upperXBound || detectedTargetPose.getY() > upperYBound) {
      DogLog.log(ntKey + "Rejected Target Pose", detectedTargetPose);
      DogLog.log(
          ntKey + "Rejected Reason",
          "Y or X is out of bounds",
          "X: " + detectedTargetPose.getX() + "," + "Y: " + detectedTargetPose.getX());

      return false;
    }

    return true;
  }
}

