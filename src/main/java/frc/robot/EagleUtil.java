package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class EagleUtil {
  protected static ArrayList<Pose2d> m_redPoses;
  protected static ArrayList<Pose2d> m_bluePoses;

  protected static ArrayList<Pose2d> m_redAlgaePoses;
  protected static ArrayList<Pose2d> m_blueAlgaePoses;

  private static double BLUE_REEF_X = Units.inchesToMeters(144 + (93.5 - 14 * 2) / 2);
  private static double BLUE_REEF_Y = Units.inchesToMeters(158.50);
  private static Pose2d BLUE_REEF = new Pose2d(BLUE_REEF_X, BLUE_REEF_Y, Rotation2d.kZero);
  private static Pose2d BLUE_REEF_INVERT = new Pose2d(-BLUE_REEF_X, -BLUE_REEF_Y, Rotation2d.kZero);
  private static double RED_REEF_X = Units.inchesToMeters(546.875 - (93.5 - 14 * 2) / 2);
  private static double RED_REEF_Y = Units.inchesToMeters(158.50);
  private static Pose2d RED_REEF = new Pose2d(RED_REEF_X, RED_REEF_Y, Rotation2d.kZero);
  private static Pose2d RED_REEF_INVERT = new Pose2d(-RED_REEF_X, -RED_REEF_Y, Rotation2d.kZero);

  private static double REEF_LENGTH = Units.inchesToMeters(35);
  private static double REEF_TO_REEF_DISTANCE = 0.33;
  private static double ROBOT_AWAY_FROM_REEF = Units.inchesToMeters(16);

  private static double X = -REEF_LENGTH - ROBOT_AWAY_FROM_REEF;
  private static double Y = REEF_TO_REEF_DISTANCE / 2;
  private static double Y_OFFSET = Units.inchesToMeters(-0.5);

  private static Pose2d[] bluePoses = new Pose2d[12];
  private static Pose2d[] redPoses = new Pose2d[12];

  private static Pose2d[] blueAlgaePoses = new Pose2d[6];
  private static Pose2d[] redAlgaePoses = new Pose2d[6];

  private static final double ALGAE_Y_OFFSET = Units.inchesToMeters(0.6);
  private static final double ALGAE_X_OFFSET = Units.inchesToMeters(0.10);
  // ALGAE_Y_OFFSET = move to left
  // ALGAE_X_OFFSET = move to right

  private static Pose2d cachedPose = null;
  private static Alliance red = DriverStation.Alliance.Red;
  static double[] redHeightReefOffsets = {
    // in meters
    0, // reef G
    0, // reef H
    0, // reef I
    0, // reef J
    0, // reef K
    0, // reef L
    0, // reef A
    0, // reef B
    0, // reef C
    0, // reef D
    0, // reef E
    0 // reef F
  };
  static double[] blueHeightReefOffsets = {
    // in meters
    0, // reef A
    0, // reef B
    0, // reef C
    0, // reef D
    0, // reef E
    0, // reef F
    0, // reef G
    0, // reef H
    0, // reef I
    0, // reef J
    0, // reef K
    0, // reef L
  };

  static double[] redAngleReefOffsets = {
    // in meters
    0, // reef G
    0, // reef H
    0, // reef I
    0, // reef J
    0, // reef K
    0, // reef L
    0, // reef A
    0, // reef B
    0, // reef C
    0, // reef D
    0, // reef E
    0 // reef F
  };
  static double[] blueAngleReefOffsets = {
    // in meters
    0, // reef A
    0, // reef B
    0, // reef C
    0, // reef D
    0, // reef E
    0, // reef F
    0, // reef G
    0, // reef H
    0, // reef I
    0, // reef J
    0, // reef K
    0, // reef L
  };

  private static List<Pose2d> coralStationPoints =
      new ArrayList<>(
          Arrays.asList(
              new Pose2d(16.05, 7.42, Rotation2d.fromDegrees(-126)), // Red Processor Side
              new Pose2d(16.05, 0.63, Rotation2d.fromDegrees(126)), // Red Non-Processor Side
              new Pose2d(1.5, 0.63, Rotation2d.fromDegrees(54)), // Blue Processor Side
              new Pose2d(1.5, 7.42, Rotation2d.fromDegrees(-54)) // Blue Non-Processor Side
              ));

  /**
   * @return returns the calculated set points
   */
  public static ArrayList<Pose2d> calculateBlueReefSetPoints() {
    if (m_bluePoses != null) {
      return new ArrayList<Pose2d>(m_bluePoses);
    }

    double[][] blueReefOffsets = {
      // in meters
      {0, 0}, // reef A
      {0, 0}, // reef B
      {0, 0}, // reef C
      {0, 0}, // reef D
      {0, 0}, // reef E
      {0, 0}, // reef F
      {0, 0}, // reef G
      {0, 0}, // reef H
      {0, 0}, // reef I
      {0, 0}, // reef J
      {0, 0}, // reef K
      {0, 0} // reef L
    };

    bluePoses[0] = new Pose2d(X, Y + Y_OFFSET, Rotation2d.kZero);
    bluePoses[1] = new Pose2d(X, -Y + Y_OFFSET, Rotation2d.kZero);

    Rotation2d sixty = Rotation2d.fromDegrees(60);

    for (int i = 2; i < bluePoses.length; i++) {
      bluePoses[i] = bluePoses[i - 2].rotateBy(sixty);
    }

    for (int i = 0; i < bluePoses.length; i++) {
      bluePoses[i] = bluePoses[i].relativeTo(BLUE_REEF_INVERT);
      bluePoses[i] =
          new Pose2d(
              bluePoses[i].getX() + blueReefOffsets[i][0],
              bluePoses[i].getY() + blueReefOffsets[i][1],
              bluePoses[i].getRotation());
    }

    DogLog.log("Caculation/Blue Reef", BLUE_REEF);
    DogLog.log("Caculation/Blue Set Points", bluePoses);
    m_bluePoses = new ArrayList<Pose2d>(Arrays.asList(bluePoses));
    return new ArrayList<Pose2d>(m_bluePoses);
  }

  /**
   * @return returns calculated setpoints
   */
  public static ArrayList<Pose2d> calculateRedReefSetPoints() {
    if (m_redPoses != null) {
      return new ArrayList<Pose2d>(m_redPoses);
    }

    redPoses[0] = new Pose2d(X, Y + Y_OFFSET, Rotation2d.kZero);
    redPoses[1] = new Pose2d(X, -Y + Y_OFFSET, Rotation2d.kZero);

    double[][] redReefOffsets = {
      // also in meters
      {0, 0}, // reef G
      {0, 0}, // reef H
      {0, 0}, // reef I
      {0, 0}, // reef J
      {0, 0}, // reef K
      {0, 0}, // reef L
      {0, 0}, // reef A
      {0, 0}, // reef B
      {0, 0}, // reef C
      {0, 0}, // reef D
      {0, 0}, // reef E
      {0, 0} // reef F
    };

    Rotation2d sixty = Rotation2d.fromDegrees(60);

    for (int i = 2; i < redPoses.length; i++) {
      redPoses[i] = redPoses[i - 2].rotateBy(sixty);
    }

    for (int i = 0; i < redPoses.length; i++) {
      redPoses[i] = redPoses[i].relativeTo(RED_REEF_INVERT);
      redPoses[i] =
          new Pose2d(
              redPoses[i].getX() + redReefOffsets[i][0],
              redPoses[i].getY() + redReefOffsets[i][1],
              redPoses[i].getRotation());
    }
    DogLog.log("Caculation/Red Reef", RED_REEF);
    DogLog.log("Caculation/Red Set Points", redPoses);

    m_redPoses = new ArrayList<Pose2d>(Arrays.asList(redPoses));
    return new ArrayList<Pose2d>(m_redPoses);
  }

  public static ArrayList<Pose2d> calculateBlueAlgaeSetPoints() {
    if (m_blueAlgaePoses != null) {
      return new ArrayList<>(m_blueAlgaePoses);
    }

    Rotation2d sixty = Rotation2d.fromDegrees(60);
    Pose2d startPose =
        new Pose2d(
            X - ALGAE_X_OFFSET,
            Y - REEF_TO_REEF_DISTANCE / 2 + ALGAE_Y_OFFSET,
            Rotation2d.kZero); // Centered on first side

    for (int i = 0; i < blueAlgaePoses.length; i++) {
      blueAlgaePoses[i] = startPose.rotateBy(sixty.times(i));
    }

    for (int i = 0; i < blueAlgaePoses.length; i++) {
      blueAlgaePoses[i] = blueAlgaePoses[i].relativeTo(BLUE_REEF_INVERT);
    }

    DogLog.log("Calculation/Blue Algae", BLUE_REEF);
    DogLog.log("Calculation/Blue Algae Setpoints", blueAlgaePoses);
    m_blueAlgaePoses = new ArrayList<>(Arrays.asList(blueAlgaePoses));
    return new ArrayList<>(m_blueAlgaePoses);
  }

  public static ArrayList<Pose2d> calculateRedAlgaeSetPoints() {
    if (m_redAlgaePoses != null) {
      return new ArrayList<>(m_redAlgaePoses);
    }

    Rotation2d sixty = Rotation2d.fromDegrees(60);
    Pose2d startPose =
        new Pose2d(
            X - ALGAE_X_OFFSET,
            Y - REEF_TO_REEF_DISTANCE / 2 + ALGAE_Y_OFFSET,
            Rotation2d.kZero); // Centered on first side

    for (int i = 0; i < redAlgaePoses.length; i++) {
      redAlgaePoses[i] = startPose.rotateBy(sixty.times(i));
    }

    for (int i = 0; i < redAlgaePoses.length; i++) {
      redAlgaePoses[i] = redAlgaePoses[i].relativeTo(RED_REEF_INVERT);
    }

    DogLog.log("Calculation/Red Algae", RED_REEF);
    DogLog.log("Calculation/Red Algae Setpoints", redAlgaePoses);
    m_redAlgaePoses = new ArrayList<>(Arrays.asList(redAlgaePoses));
    return new ArrayList<>(m_redAlgaePoses);
  }

  private static Pose2d getNearestReefPoint(Pose2d pose) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return pose.nearest(FieldConstants.blueReefSetpointList);
    } else {
      return pose.nearest(FieldConstants.redReefSetpointList);
    }
  }

  public static Pose2d getNearestAlgaePoint(Pose2d pose) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return pose.nearest(FieldConstants.blueAlgaeSetpointList);
    } else {
      return pose.nearest(FieldConstants.redAlgaeSetpointList);
    }
  }

  public static boolean isHighAlgae(Pose2d pose) {
    Pose2d nearest = getNearestAlgaePoint(pose);
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      int index = FieldConstants.blueAlgaeSetpointList.indexOf(nearest);
      return (index % 2 == 0);
    } else {
      int index = FieldConstants.redAlgaeSetpointList.indexOf(nearest);
      return (index % 2 == 1);
    }
  }

  public static Pose2d getClosestL1Back(Pose2d pose) {
    Pose2d targetPose;
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      targetPose = pose.nearest(FieldConstants.blueAlgaeSetpointList);
    } else {
      targetPose = pose.nearest(FieldConstants.redAlgaeSetpointList);
    }
    Rotation2d targetRotation = new Rotation2d(targetPose.getRotation().getRadians() + Math.PI);
    targetPose = new Pose2d(targetPose.getX(), targetPose.getY(), targetRotation);
    return targetPose;
  }

  /**
   * @param pose the pose to compare to
   * @return cached Pose2d
   */
  public static Pose2d getCachedReefPose(Pose2d pose) {
    if (cachedPose == null) {
      cachedPose = getNearestReefPoint(pose);
    }
    return cachedPose;
  }

  public static void clearCachedPose() {
    cachedPose = null;
  }

  /**
   * @param poseA first pose
   * @param poseB second pose
   * @return distance in between
   */
  public static double getDistanceBetween(Pose2d poseA, Pose2d poseB) {
    return poseA.getTranslation().getDistance(poseB.getTranslation());
  }

  public static class PoseComparator implements Comparator<Pose2d> {
    public Pose2d robotPose;

    public PoseComparator(Pose2d robotPose) {
      this.robotPose = robotPose;
    }

    public int compare(Pose2d poseA, Pose2d poseB) {
      double distA = poseA.getTranslation().getDistance(robotPose.getTranslation());
      double distB = poseB.getTranslation().getDistance(robotPose.getTranslation());
      if (distA > distB) {
        return 1;
      } else if (distA == distB) {
        return 0;
      }
      return -1;
    }
  }

  /**
   * @param pose the pose to compare
   * @param n how many from closest
   * @return the pose which is nth away from closest
   */
  public static Pose2d closestReefSetPoint(Pose2d pose, int n) {
    n = MathUtil.clamp(n, 0, 23);
    if (isRedAlliance()) {
      ArrayList<Pose2d> red = calculateRedReefSetPoints();
      red.sort(new PoseComparator(pose));
      return red.get(n);
    }

    ArrayList<Pose2d> blue = calculateBlueReefSetPoints();
    blue.sort(new PoseComparator(pose));
    return blue.get(n);
  }

  public static int findClosestReefIndex(Pose2d pose) {
    ArrayList<Pose2d> reefSetPoints = null;
    if (isRedAlliance()) {
      reefSetPoints = calculateRedReefSetPoints();
    } else {
      reefSetPoints = calculateBlueReefSetPoints();
    }

    double minimumDistance = Double.MAX_VALUE;
    int minIndex = 0;
    for (int i = 0; i < reefSetPoints.size(); i++) {
      double distance = getDistanceBetween(pose, reefSetPoints.get(i));
      if (distance < minimumDistance) {
        minimumDistance = distance;
        minIndex = i;
      }
    }
    return minIndex;
  }

  /**
   * @return if your on red alliance
   */
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == red;
  }

  /**
   * @param alert what alert to trigger
   * @return run the command
   */
  public static Command triggerAlert(Alert alert) {
    return Commands.runOnce(() -> alert.set(true));
  }

  /**
   * @param alert what alert to untrigger
   * @return run the command
   */
  public static Command detriggerAlert(Alert alert) {
    return Commands.runOnce(() -> alert.set(false));
  }

  public static Pose2d blueCoralStationProcessorSide = new Pose2d(1.0, 0.27, Rotation2d.kZero);
  public static Pose2d blueCoralStationNonProcessorSide = new Pose2d(1.0, 7.313, Rotation2d.kZero);
  public static Pose2d redCoralStationProcessorSide = new Pose2d(16.5, 0.27, Rotation2d.kZero);
  public static Pose2d redCoralStationNonProcessorSide = new Pose2d(16.5, 7.415, Rotation2d.kZero);

  /**
   * @return the pose for your processor
   */
  public static Pose2d getProcessorForAlliance() {
    if (isRedAlliance()) {
      return redCoralStationProcessorSide;
    }
    return blueCoralStationProcessorSide;
  }

  /**
   * @return the pose for your non-processor
   */
  public static Pose2d getNonProcessorForAlliance() {
    if (isRedAlliance()) {
      return redCoralStationNonProcessorSide;
    }
    return blueCoralStationNonProcessorSide;
  }

  /**
   * @param pose compare to this pose
   * @return the closest one's pose
   */
  public static Pose2d getClosetStationGen(Pose2d pose) {
    if (getDistanceBetween(pose, getNonProcessorForAlliance())
        < getDistanceBetween(pose, getProcessorForAlliance())) {
      return getNonProcessorForAlliance();
    }
    return getProcessorForAlliance();
  }

  public static double getOffsetArmAngle(Pose2d robotPose) {
    double armAngle = 4.0;

    int reefIndex = findClosestReefIndex(robotPose);
    if (isRedAlliance()) {
      return armAngle + redAngleReefOffsets[reefIndex];
    }
    return armAngle + blueAngleReefOffsets[reefIndex];
  }

  public static double getOffsetElevatorHeight(Pose2d robotPose) {
    double elevatorHeight = 4.0;

    int reefIndex = findClosestReefIndex(robotPose);

    if (isRedAlliance()) {
      return elevatorHeight + redHeightReefOffsets[reefIndex];
    }
    return elevatorHeight + blueHeightReefOffsets[reefIndex];
  }

  public static Pose2d getClosestLeftReef(Pose2d pose) {
    int closestReef = findClosestReefIndex(pose);
    if (closestReef % 2 > 0) {
      closestReef -= 1;
    }

    if (isRedAlliance()) {
      calculateRedReefSetPoints();
      return redPoses[closestReef];
    } else {
      calculateBlueReefSetPoints();
      return bluePoses[closestReef];
    }
  }

  public static Pose2d getClosestLeftReefBack(Pose2d pose) {
    int closestReef = findClosestReefIndex(pose);
    if (closestReef % 2 > 0) {
      closestReef -= 1;
    }

    Pose2d targetPose;
    if (isRedAlliance()) {
      calculateRedReefSetPoints();
      targetPose = redPoses[closestReef];
    } else {
      calculateBlueReefSetPoints();
      targetPose = bluePoses[closestReef];
    }
    Rotation2d targetRotation = new Rotation2d(targetPose.getRotation().getRadians() + Math.PI);
    targetPose = new Pose2d(targetPose.getX(), targetPose.getY(), targetRotation);
    return targetPose;
  }

  public static Pose2d getClosestRightReefBack(Pose2d pose) {
    int closestReef = findClosestReefIndex(pose);
    if (closestReef % 2 == 0) {
      closestReef += 1;
    }

    Pose2d targetPose;
    if (isRedAlliance()) {
      calculateRedReefSetPoints();
      targetPose = redPoses[closestReef];
    } else {
      calculateBlueReefSetPoints();
      targetPose = bluePoses[closestReef];
    }
    Rotation2d targetRotation = new Rotation2d(targetPose.getRotation().getRadians() + Math.PI);
    targetPose = new Pose2d(targetPose.getX(), targetPose.getY(), targetRotation);
    return targetPose;
  }

  public static Pose2d getClosestCoralStation(Pose2d pose) {
    return pose.nearest(coralStationPoints);
  }

  public static double getRotationCenterReef(Pose2d pose) {
    Pose2d reef;

    if (isRedAlliance()) {
      reef = RED_REEF;
    } else {
      reef = BLUE_REEF;
    }

    return reef.getTranslation().minus(pose.getTranslation()).getAngle().getDegrees();
  }

  public static Pose2d aligntobarge(Pose2d pose) {
    if (FieldConstants.HALF_THE_FIELD > pose.getX()) {
      Pose2d poseBlue =
          new Pose2d(
              FieldConstants.SCORE_NET_BLUE_SIDE_X, pose.getY(), Rotation2d.fromDegrees(-44));
      return poseBlue;
    }
    Pose2d poseRed =
        new Pose2d(FieldConstants.SCORE_NET_RED_SIDE_X, pose.getY(), Rotation2d.fromDegrees(44));
    return poseRed;
  }

  public static Pose2d getClosestRightReef(Pose2d pose) {
    int closestReef = findClosestReefIndex(pose);
    if (closestReef % 2 == 0) {
      closestReef += 1;
    }

    if (isRedAlliance()) {
      calculateRedReefSetPoints();
      return redPoses[closestReef];
    } else {
      calculateBlueReefSetPoints();
      return bluePoses[closestReef];
    }
  }
}
