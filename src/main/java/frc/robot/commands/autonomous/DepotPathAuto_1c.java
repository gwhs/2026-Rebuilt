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

public class DepotPathAuto_1c extends SequentialCommandGroup {
  private Pose2d getScorePose(PathPlannerPath path) {
    Pose2d score;
    if (EagleUtil.isRedAlliance()) {
      score =
          new Pose2d(
              path.flipPath().getPoint(0).position,
              path.flipPath().getIdealStartingState().rotation());
    } else {
      score = new Pose2d(path.getPoint(0).position, path.getIdealStartingState().rotation());
    }
    return score;
  }

  public DepotPathAuto_1c(SwerveSubsystem drivetrain, ShooterSubsystem shooter) {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      PathPlannerPath startingPath = PathPlannerPath.fromChoreoTrajectory("Start_Depot");
      PathPlannerPath depotPath = PathPlannerPath.fromChoreoTrajectory("Depot_Depot");
      PathPlannerPath scorePath = PathPlannerPath.fromChoreoTrajectory("Depot_Score");
      PathPlannerPath climbPath = PathPlannerPath.fromChoreoTrajectory("Score_Climb");
      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory("PATH NAME");

      Pose2d startingPose =
          new Pose2d(
              startingPath.getPoint(0).position, startingPath.getIdealStartingState().rotation());
      Pose2d score;
      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(startingPath),
          AutoBuilder.followPath(depotPath),
          AutoBuilder.followPath(scorePath).deadlineFor(shooter.runVelocity(80)),
          Commands.waitSeconds(6)
              .deadlineFor(drivetrain.driveToPose(() -> getScorePose(climbPath))),
          AutoBuilder.followPath(climbPath).deadlineFor(shooter.runVelocity(0)));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
