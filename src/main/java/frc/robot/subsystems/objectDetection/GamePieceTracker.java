package frc.robot.subsystems.objectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedList;
import java.util.Optional;

public class GamePieceTracker {

  public static final double TIMESTAMP_THRESHOLD = 1.0;
  public static final double NEW_TARGET_THRESHOLD = 0.33;

  private static final LinkedList<Target> targets = new LinkedList<>();

  public static void addTarget(double timestamp, Pose2d pose) {
    Optional<Pose2d> currentFocusedGamePiece = getGamePiece();

    if (currentFocusedGamePiece.isEmpty()) {
      targets.add(new Target(timestamp, pose));
      return;
    }

    Pose2d currentFocusedGamePiecePosition = currentFocusedGamePiece.get();
    double distance =
        currentFocusedGamePiecePosition.getTranslation().getDistance(pose.getTranslation());

    if (distance < NEW_TARGET_THRESHOLD) {
      targets.add(new Target(timestamp, pose));
    } else {
      targets.clear();
      targets.add(new Target(timestamp, pose));
    }
  }

  public static Optional<Pose2d> getGamePiece() {
    double currentTime = Timer.getFPGATimestamp(); // gets the current time

    // Remove targets that are older than the threshold
    if (targets.isEmpty()) {
      return Optional.empty();
    }

    while (targets.peek() != null
        && currentTime - targets.peek().timestamp() >= TIMESTAMP_THRESHOLD) {
      targets.pollFirst();
    }

    // calculates the average position over the last specified time, then returns result
    double avgx = 0;
    double avgy = 0;
    double n = 0;

    for (int i = 0; i < targets.size(); i++) {
      Target target = targets.get(i);

      double timeStamp = target.timestamp();
      if (currentTime - timeStamp >= TIMESTAMP_THRESHOLD) {
        continue;
      }

      avgx += target.pose().getX();
      avgy += target.pose().getY();
      n++;
    }

    if (n == 0) {
      // no valid game piece in sight
      return Optional.empty();
    }

    avgx /= n;
    avgy /= n;

    Pose2d result = new Pose2d(avgx, avgy, Rotation2d.kZero);
    return Optional.of(result);
  }

  public void clearTargets() {
    targets.clear();
  }
}
