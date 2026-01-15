package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.swerve.SwerveSubsystem.RotationTarget;

public class EagleUtil {

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  public static double getTarget(RotationTarget target, Pose2d pose) {
    Translation2d alignmentTarget;
    if (target == RotationTarget.PASSING_DEPOT_SIDE) {
      if (isRedAlliance()) {
        alignmentTarget = FieldConstants.RED_DEPOT;

        return alignmentTarget.minus(pose.getTranslation()).getAngle().getDegrees();
      }
      alignmentTarget = FieldConstants.BLUE_DEPOT;

      return alignmentTarget.minus(pose.getTranslation()).getAngle().getDegrees();
    }
    if (isRedAlliance()) {
      alignmentTarget = FieldConstants.RED_OUTPOST;

      return alignmentTarget.minus(pose.getTranslation()).getAngle().getDegrees();
    }
    alignmentTarget = FieldConstants.BLUE_OUTPOST;
    return alignmentTarget.minus(pose.getTranslation()).getAngle().getDegrees();
=======

public class EagleUtil {
  public static double getRobotTargetAngle(Pose2d robotpose, Translation2d target) {
    return target.minus(robotpose.getTranslation()).getAngle().getDegrees();
  }

  public static double getRobotTargetDistance(Pose2d robotpose, Translation2d target) {
    return target.getDistance(robotpose.getTranslation());
>>>>>>> f80bdd4 (added getRobotTargetAngle and getRobotTargetDistance methood (#20))
  }
}
