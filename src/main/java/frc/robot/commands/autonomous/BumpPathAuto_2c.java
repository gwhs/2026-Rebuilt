package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class BumpPathAuto_2c extends SequentialCommandGroup {
  public BumpPathAuto_2c(ShooterSubsystem shooter, boolean mirror) {

    /* All your code should go inside this try-catch block */
    try {

      PathPlannerPath neutral_score = PathPlannerPath.fromChoreoTrajectory("Neutral_Score");
      PathPlannerPath neutral = PathPlannerPath.fromChoreoTrajectory("Neutral");
      PathPlannerPath score_neutral = PathPlannerPath.fromChoreoTrajectory("Score_Neutral");

      if (mirror) {
        neutral_score = neutral_score.mirrorPath();
        neutral = neutral.mirrorPath();
        score_neutral = score_neutral.mirrorPath();
      }

      Pose2d startingPose =
          new Pose2d(
              score_neutral.getPoint(0).position, score_neutral.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(score_neutral),
          AutoBuilder.followPath(neutral),
          AutoBuilder.followPath(neutral_score),
          shooter.runVelocity(5000),
          Commands.waitSeconds(6.0),
          shooter.runVelocity(0),
          AutoBuilder.followPath(score_neutral),
          AutoBuilder.followPath(neutral),
          AutoBuilder.followPath(neutral_score),
          shooter.runVelocity(5000),
          Commands.waitSeconds(7.0), // score
          shooter.runVelocity(0),
          AutoBuilder.followPath(score_neutral));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
