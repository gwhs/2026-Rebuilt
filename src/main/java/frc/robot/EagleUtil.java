package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.swerve.SwerveSubsystem.RotationTarget;

public class EagleUtil {

  private static double RED_DEPOTX = Units.inchesToMeters(2.014);
  private static double RED_DEPOTY = Units.inchesToMeters(6.704);
  private static double BLUE_DEPOTX = Units.inchesToMeters(9.604);
  private static double BLUE_DEPOTY = Units.inchesToMeters(7.602);
  private static double RED_OUTPOSTX = Units.inchesToMeters(1.845);
  private static double RED_OUTPOSTY = Units.inchesToMeters(1.976);
  private static double BLUE_OUTPOSTX = Units.inchesToMeters(9.535);
  private static double BLUE_OUTPOSTY = Units.inchesToMeters(0.888);
  private static Translation2d RED_DEPOT = new Translation2d(RED_DEPOTX, RED_DEPOTY);
  private static Translation2d BLUE_DEPOT = new Translation2d(BLUE_DEPOTX, BLUE_DEPOTY);
  private static Translation2d RED_OUTPOST = new Translation2d(RED_OUTPOSTX, RED_OUTPOSTY);
  private static Translation2d BLUE_OUTPOST = new Translation2d(BLUE_OUTPOSTX, BLUE_OUTPOSTY);

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  public static double getTarget(RotationTarget target, Pose2d pose) {
    Translation2d alignmentTarget;
    if (target == RotationTarget.PASSING_DEPOT_SIDE) {
      if (isRedAlliance()) {
        alignmentTarget = RED_DEPOT;

        return alignmentTarget.minus(pose.getTranslation()).getAngle().getDegrees();
      }
      alignmentTarget = BLUE_DEPOT;

      return alignmentTarget.minus(pose.getTranslation()).getAngle().getDegrees();
    }
    if (isRedAlliance()) {
      alignmentTarget = RED_OUTPOST;

      return alignmentTarget.minus(pose.getTranslation()).getAngle().getDegrees();
    }
    alignmentTarget = BLUE_OUTPOST;
    return alignmentTarget.minus(pose.getTranslation()).getAngle().getDegrees();
  }
}
