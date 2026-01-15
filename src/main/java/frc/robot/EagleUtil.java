package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class EagleUtil {

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  public static double getrotationalhub(Pose2d pose) {
    Translation2d Hub;

    if (isRedAlliance()) {
      Hub = FieldConstants.RED_HUB;
    } else {
      Hub = FieldConstants.BLUE_HUB;
    }
    return Hub.minus(pose.getTranslation()).getAngle().getDegrees();
  }
}
