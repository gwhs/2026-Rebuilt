package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class EagleUtil {
  public static double getRobotTargetAngle(Pose2d robotpose, Translation2d target) {
    return target.minus(robotpose.getTranslation()).getAngle().getDegrees();
  }

  public static double getRobotTargetDistance(Pose2d robotpose, Translation2d target) {
    return target.getDistance(robotpose.getTranslation());
  }
}
