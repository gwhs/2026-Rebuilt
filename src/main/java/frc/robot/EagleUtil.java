package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  public static Pose2d calcAimpoint(
      Pose2d robotPose, Pose2d newRobotPose, Translation2d target, ChassisSpeeds robot) {
    double dis = getRobotTargetDistance(newRobotPose, target);
    double x = robotPose.getX() + target.getX() - newRobotPose.getX() + getFuelDx(robot, dis);
    double y = robotPose.getY() + target.getY() - newRobotPose.getY() + getFuelDy(robot, dis);
    Pose2d aimpoint = new Pose2d(x, y, Rotation2d.kZero);
    return aimpoint;
  }

  public static double getFuelDx(ChassisSpeeds robot, double distanceToTarget) {
    return robot.vxMetersPerSecond * distanceToTarget / FieldConstants.fuelSpeed;
  }

  public static double getFuelDy(ChassisSpeeds robot, double distanceToTarget) {
    return robot.vyMetersPerSecond * distanceToTarget / FieldConstants.fuelSpeed;
  }



    public static List<Translation2d> generateHalfCirclePoints(double centerX, double centerY, double radius, double angleStep) {
        List<Translation2d> points = new ArrayList<>();
        
	for (double angle = 0; angle <= 180; angle += angleStep) {
            double angleInRadians = Math.toRadians(angle);

            // Calculate point coordinates
            double x = centerX + radius * Math.cos(angleInRadians);
            double y = centerY + radius * Math.sin(angleInRadians);

            points.add(new Translation2d(x, y));
        }
        return points;
    } }

