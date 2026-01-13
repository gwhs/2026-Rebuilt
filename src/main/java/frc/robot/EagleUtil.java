package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class EagleUtil {
  private static double BLUE_HUBX = Units.inchesToMeters(469.11);
  private static double BLUE_HUBY = Units.inchesToMeters(158.84);
  static Translation2d BLUE_HUB = new Translation2d(BLUE_HUBX, BLUE_HUBY);

  private static double RED_HUBX = Units.inchesToMeters(182.11);
  private static double RED_HUBY = Units.inchesToMeters(158.84);
  static Translation2d RED_HUB = new Translation2d(RED_HUBX, RED_HUBY);

  private static double BLUE_TOWERX = Units.inchesToMeters(607.66);
  private static double BLUE_TOWERY = Units.inchesToMeters(170.22);
  static Translation2d BLUE_TOWER = new Translation2d(BLUE_TOWERX, BLUE_TOWERY);

  private static double RED_TOWERX = Units.inchesToMeters(43.56);
  private static double RED_TOWERY = Units.inchesToMeters(170.22);
  static Translation2d RED_TOWER = new Translation2d(RED_TOWERX, RED_TOWERY);

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  public static double getRotationalHub(Pose2d pose) {
    Translation2d Hub;

    if (isRedAlliance()) {
      Hub = RED_HUB;
    } else {
      Hub = BLUE_HUB;
    }
    return Hub.minus(pose.getTranslation()).getAngle().getDegrees();
  }
}
