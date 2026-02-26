package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;

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
    double c = 1.075 + (distanceToTarget * 0.005); // constant to fix inefficiency
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

  public static double getFuelTimeInAir(double distanceToTarget) {
    // eqation based on simulation data points and adjustments, may be adjusted later for better
    // performance
    return 0.63117 * Math.sin(0.23124 * distanceToTarget - 1.6501) + 1.87679;
  }

  public static Command shootInSim(SwerveSubsystem drivetrain) {
    return Commands.sequence(
            Commands.waitSeconds(0.1),
            Commands.runOnce(
                () -> {
                  if (RobotBase.isSimulation()) {
                    Pose2d shotPos = EagleUtil.getShooterPos(drivetrain.getCachedState().Pose);
                    Translation3d initPosition =
                        new Translation3d(shotPos.getX(), shotPos.getY(), 0.635);
                    Pose2d tar = drivetrain.getCachedVirtualTarget();
                    double dist = EagleUtil.getRobotTargetDistance(shotPos, tar);
                    double t = EagleUtil.getFuelTimeInAir(dist);
                    double v = EagleUtil.getShooterVelocity(dist);
                    DogLog.log("velocity of fuel", v);
                    DogLog.log("distance to tar", dist);
                    DogLog.log("fuelTimeInAir", t);

                    ChassisSpeeds robotVelocityChassis =
                        ChassisSpeeds.fromRobotRelativeSpeeds(
                            drivetrain.getCachedState().Speeds,
                            drivetrain.getCachedState().Pose.getRotation());
                    double robotDx = robotVelocityChassis.vxMetersPerSecond;
                    double robotDy = robotVelocityChassis.vyMetersPerSecond;

                    double a = drivetrain.getCachedState().Pose.getRotation().getRadians();
                    Translation3d initVelocity =
                        new Translation3d(
                            (v * Math.cos(FieldConstants.shooterAngleRadian) * Math.cos(a))
                                + robotDx, // x
                            (v * Math.cos(FieldConstants.shooterAngleRadian) * Math.sin(a))
                                + robotDy, // y
                            v * Math.sin(FieldConstants.shooterAngleRadian)); // z
                    FuelSim.getInstance()
                        .spawnFuel(
                            initPosition,
                            initVelocity); // spawns a fuel with a given position and velocity (both
                    // field centric, represented as vectors by Translation3d)
                  }
                }))
        .repeatedly();
  }

  public static Pose2d getClimbPose(Pose2d robotPose) {
    return robotPose.nearest(FieldConstants.CLIMBPOSE);
  }
}
