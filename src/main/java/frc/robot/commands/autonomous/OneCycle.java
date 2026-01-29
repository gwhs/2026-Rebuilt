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

public class OneCycle extends SequentialCommandGroup {
  public enum Routine {
    BUMP,
    TRENCH
  }

  public OneCycle(
      ShooterSubsystem shooter, SwerveSubsystem drivetrain, boolean mirror, Routine routine) {

    /* All your code should go inside this try-catch block */
    try {
      // Load Paths
      PathPlannerPath cycle;
      PathPlannerPath climb;
      String pathprefix = "";

      if (routine == Routine.BUMP) {
        pathprefix = "B_";
      } else if (routine == Routine.TRENCH) {
        pathprefix = "T_";
      }

      if (mirror) {
        cycle = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Cycle").mirrorPath();
        climb = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Climb_Mirrored");
      } else {
        cycle = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Cycle");
        climb = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Climb");
      }
      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory(pathprefix + "PATH
      // NAME");

      Pose2d startingPose =
          new Pose2d(cycle.getPoint(0).position, cycle.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(cycle).deadlineFor(shooter.runVelocity(80)),
          Commands.waitSeconds(6)
              .deadlineFor(drivetrain.driveToPose(() -> getScorePose(() -> cycle))),
          AutoBuilder.followPath(climb).deadlineFor(shooter.runVelocity(0)),
          Commands.idle());

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
