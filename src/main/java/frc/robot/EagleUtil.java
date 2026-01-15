package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class EagleUtil {
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
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
}
