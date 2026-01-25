package frc.robot;

import java.util.Vector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.shooter.ShooterIOReal;

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

  public static double getShooterVelocity(double distanceToTarget)
  {
    double v = Math.sqrt((FieldConstants.gravitationalAcc * distanceToTarget * distanceToTarget) 
    / (2 * Math.cos(FieldConstants.shooterAngle) * Math.cos(FieldConstants.shooterAngle) 
    * (distanceToTarget * Math.tan(FieldConstants.shooterAngle) - (FieldConstants.hubHeight - FieldConstants.shooterHeight))));
    return v;
  }

  public static double rpmToVelocity(double rpm)
  {
    double flywheelRPM = rpm * FieldConstants.motorToFlywheelGearRatio;
    return Math.PI * FieldConstants.flywheelDiameter * flywheelRPM;
  }

  public static double velocityToRPM(double v)
  {
    double flywheelRPM = v / Math.PI / FieldConstants.flywheelDiameter;
    return flywheelRPM / FieldConstants.motorToFlywheelGearRatio;
  }

  public static double sotfVector(Translation2d target, Pose2d robotPosFuture)
  {
    Translation2d tar = target.minus(robotPosFuture.getTranslation());
    double dist = tar.getNorm();
    double shootVelocity = getShooterVelocity(dist);
    Translation2d tarVec = tar.div(dist).times(shootVelocity);
    Translation2d robotVelocityFieldCentric = new Translation2d(0, 0);
    Translation2d shotVec = tarVec.minus(robotVelocityFieldCentric);
    double shooterRPM = 0; //update
    return shooterRPM;
  }
}
