package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.swerve.SwerveSubsystem.RotationTarget;

public class EagleUtil {

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  public static double getRotationalHub(Pose2d pose) {
    Translation2d hub;

    if (isRedAlliance()) {
      hub = FieldConstants.RED_HUB;
    } else {
      hub = FieldConstants.BLUE_HUB;
    }
    return hub.minus(pose.getTranslation()).getAngle().getDegrees();
  }

  public static boolean isInAllianceZone(Pose2d robotPose) {
    if (isRedAlliance()) {
      return robotPose.getX() > FieldConstants.ALLIANCE_ZONE_LINE_RED;
    } else {
      return robotPose.getX() < FieldConstants.ALLIANCE_ZONE_LINE_BLUE;
    }
  }

  public static boolean isInOpponentAllianceZone(Pose2d robotPose) {
    if (isRedAlliance()) {
      return robotPose.getX() < FieldConstants.ALLIANCE_ZONE_LINE_BLUE;
    } else {
      return robotPose.getX() > FieldConstants.ALLIANCE_ZONE_LINE_RED;
    }
  }

  public static boolean isInNeutralZone(Pose2d robotPose) {
    return (robotPose.getX() >= FieldConstants.ALLIANCE_ZONE_LINE_BLUE
        && robotPose.getX() <= FieldConstants.ALLIANCE_ZONE_LINE_RED);
  }

  public static double getRobotTargetAngle(RotationTarget target, Pose2d pose) {
    Translation2d alignmentTarget;
    if (target == RotationTarget.PASSING_DEPOT_SIDE) {
      if (isRedAlliance()) {
        alignmentTarget = FieldConstants.RED_DEPOT_PASSING;
        return getTargetAngle(pose, alignmentTarget);
      }
      alignmentTarget = FieldConstants.BLUE_DEPOT_PASSING;
      return getTargetAngle(pose, alignmentTarget);
    }
    if (target == RotationTarget.PASSING_OUTPOST_SIDE) {
      if (isRedAlliance()) {
        alignmentTarget = FieldConstants.RED_OUTPOST_PASSING;
        getTargetAngle(pose, alignmentTarget);
      }
      alignmentTarget = FieldConstants.BLUE_OUTPOST_PASSING;
      return getTargetAngle(pose, alignmentTarget);
    }
    return 0.0;
  }

  private static double getTargetAngle(Pose2d robotpose, Translation2d target) {
    return target.minus(robotpose.getTranslation()).getAngle().getDegrees();
  }

  public static double getRobotTargetDistance(Pose2d robotpose, Translation2d target) {
    return target.getDistance(robotpose.getTranslation());
  }
}
