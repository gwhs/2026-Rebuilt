package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

  public static boolean isOnOutpostSide(Pose2d robotPose) {
    if (isRedAlliance()) {
      return robotPose.getY() > FieldConstants.FIELD_WIDTH / 2;
    }
    return robotPose.getY() < FieldConstants.FIELD_WIDTH / 2;
  }

  public static boolean isOnDepotSide(Pose2d robotPose) {
    if (isRedAlliance()) {
      return robotPose.getY() < FieldConstants.FIELD_WIDTH / 2;
    }
    return robotPose.getY() > FieldConstants.FIELD_WIDTH / 2;
  }

  public static double getRobotTargetAngle(Pose2d robotpose, Pose2d target) {
    return target.getTranslation().minus(robotpose.getTranslation()).getAngle().getDegrees();
  }

  public static double getRobotTargetAngleRadian(Pose2d robotpose, Pose2d target) {
    return target.getTranslation().minus(robotpose.getTranslation()).getAngle().getRadians();
  }

  public static boolean isOnBump(Pose2d robotPose) {
    return (robotPose.getX() >= FieldConstants.BLUE_BUMP_X1
            && robotPose.getX() <= FieldConstants.BLUE_BUMP_X2)
        || (robotPose.getX() >= FieldConstants.RED_BUMP_X1
            && robotPose.getX() <= FieldConstants.RED_BUMP_X2);
  }

  public static double getRobotTargetAngle(Pose2d robotpose, Translation2d target) {
    return target.minus(robotpose.getTranslation()).getAngle().getDegrees();
  }

  public static double getRobotTargetDistance(Pose2d robotpose, Translation2d target) {
    return target.getDistance(robotpose.getTranslation());
  }

  public static double getRobotTargetDistance(Pose2d robotpose, Pose2d target) {
    return target.getTranslation().getDistance(robotpose.getTranslation());
  }

  public static Pose2d calcAimpoint(Pose2d robotPose, Pose2d newRobotPose, Translation2d target) {
    double x = robotPose.getX() + target.getX() - newRobotPose.getX();
    double y = robotPose.getY() + target.getY() - newRobotPose.getY();
    Pose2d aimpoint = new Pose2d(x, y, Rotation2d.kZero);
    return aimpoint;
  }

  public static double getShooterVelocity(double distanceToTarget) {
    double v =
        Math.sqrt(
            (FieldConstants.gravitationalAcc * distanceToTarget * distanceToTarget)
                / (2
                    * Math.cos(FieldConstants.shooterAngleRadian)
                    * Math.cos(FieldConstants.shooterAngleRadian)
                    * (distanceToTarget * Math.tan(FieldConstants.shooterAngleRadian)
                        - (FieldConstants.hubHeight - FieldConstants.shooterHeight))));
    double c = 1; // constance to fix inefficiency
    return v * c;
  }

  public static Translation2d getRobotTarget(Pose2d robotPose) {
    if (isRedAlliance()) {
      if (isInAllianceZone(robotPose)) {
        return FieldConstants.RED_HUB;
      } else {
        if (isOnOutpostSide(robotPose)) {
          return FieldConstants.RED_OUTPOST_PASSING;
        } else {
          return FieldConstants.RED_DEPOT_PASSING;
        }
      }
    } else {
      if (isInAllianceZone(robotPose)) {
        return FieldConstants.BLUE_HUB;
      } else {
        if (isOnOutpostSide(robotPose)) {
          return FieldConstants.BLUE_OUTPOST_PASSING;
        } else {
          return FieldConstants.BLUE_DEPOT_PASSING;
        }
      }
    }
  }

  public static Pose2d getShooterPos(Pose2d robotPos) {
    double x =
        robotPos.getX()
            - (FieldConstants.robotCenterShooterDist
                * Math.cos(robotPos.getRotation().getRadians()));
    double y = robotPos.getY();
    return new Pose2d(x, y, robotPos.getRotation());
  }

  public static double getFuelFlyTime(double dis) {
    return (-0.0020 * Math.pow(dis, 3))
        + (0.0311744 * Math.pow(dis, 2))
        + (-0.0124719 * dis)
        + 1.18962;
  }
}