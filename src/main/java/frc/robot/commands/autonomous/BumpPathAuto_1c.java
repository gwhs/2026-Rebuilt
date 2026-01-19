package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class BumpPathAuto_1c extends SequentialCommandGroup {
  public BumpPathAuto_1c(SwerveSubsystem drivetrain, ShooterSubsystem shooter, boolean mirror) {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      PathPlannerPath climb = PathPlannerPath.fromChoreoTrajectory("Climb");
      PathPlannerPath neutral_score = PathPlannerPath.fromChoreoTrajectory("Neutral_Score");
      PathPlannerPath neutral = PathPlannerPath.fromChoreoTrajectory("Neutral");
      PathPlannerPath score_neutral = PathPlannerPath.fromChoreoTrajectory("Score_Neutral");

      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory("PATH NAME");

      if (mirror) {
        neutral = neutral.mirrorPath();
        neutral_score = neutral_score.mirrorPath();
        score_neutral = score_neutral.mirrorPath();
        climb = PathPlannerPath.fromChoreoTrajectory("Climb_Mirrored");
      }

      Pose2d startingPose =
          new Pose2d(
              score_neutral.getPoint(0).position, score_neutral.getIdealStartingState().rotation());
      Pose2d score =
          new Pose2d(
              score_neutral.getPoint(0).position, score_neutral.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(score_neutral),
          AutoBuilder.followPath(neutral),
          AutoBuilder.followPath(neutral_score),
          shooter.runVelocity(80),
          Commands.waitSeconds(6.0).alongWith(drivetrain.driveToPose(() -> score)),
          shooter.runVelocity(0),
          AutoBuilder.followPath(climb),
          // stow/protect ground intake
          Commands.waitSeconds(3.0));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
