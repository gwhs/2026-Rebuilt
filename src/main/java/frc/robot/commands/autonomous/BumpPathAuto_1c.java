package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.EagleUtil;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.Supplier;

public class BumpPathAuto_1c extends SequentialCommandGroup {
  public BumpPathAuto_1c(SwerveSubsystem drivetrain, ShooterSubsystem shooter, boolean mirror) {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      PathPlannerPath climb;
      PathPlannerPath neutral_score;
      PathPlannerPath neutral;
      PathPlannerPath score_neutral;

      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory("PATH NAME");

      if (mirror) {
        neutral = PathPlannerPath.fromChoreoTrajectory("Neutral").mirrorPath();
        neutral_score = PathPlannerPath.fromChoreoTrajectory("Neutral_Score").mirrorPath();
        score_neutral = PathPlannerPath.fromChoreoTrajectory("Score_Neutral").mirrorPath();
        climb = PathPlannerPath.fromChoreoTrajectory("Climb_Mirrored");
      } else {
        neutral = PathPlannerPath.fromChoreoTrajectory("Neutral");
        neutral_score = PathPlannerPath.fromChoreoTrajectory("Neutral_Score");
        score_neutral = PathPlannerPath.fromChoreoTrajectory("Score_Neutral");
        climb = PathPlannerPath.fromChoreoTrajectory("Climb");
      }

      Pose2d startingPose =
          new Pose2d(
              score_neutral.getPoint(0).position, score_neutral.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose),
          AutoBuilder.followPath(score_neutral),
          AutoBuilder.followPath(neutral),
          AutoBuilder.followPath(neutral_score),
          shooter.runVelocity(80),
          Commands.waitSeconds(6.0)
              .deadlineFor(drivetrain.driveToPose(() -> getScorePose(() -> score_neutral))),
          shooter.runVelocity(0),
          AutoBuilder.followPath(climb),
          // stow/protect ground intake
          Commands.waitSeconds(3.0));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }

  private Pose2d getScorePose(Supplier<PathPlannerPath> path) {
    Pose2d score;
    if (EagleUtil.isRedAlliance()) {
      score =
          new Pose2d(
              path.get().flipPath().getPoint(0).position,
              path.get().flipPath().getIdealStartingState().rotation());
    } else {
      score =
          new Pose2d(
              path.get().getPoint(0).position, path.get().getIdealStartingState().rotation());
    }
    return score;
  }
}
