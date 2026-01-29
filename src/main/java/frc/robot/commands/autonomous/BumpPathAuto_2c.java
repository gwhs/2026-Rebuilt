package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.EagleUtil;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.Supplier;

public class BumpPathAuto_2c extends SequentialCommandGroup {
  public BumpPathAuto_2c(SwerveSubsystem drivetrain, ShooterSubsystem shooter, boolean mirror) {

    /* All your code should go inside this try-catch block */
    try {

      PathPlannerPath cyclePath;
      PathPlannerPath cycle2Path;

      if (mirror) {
        cyclePath = PathPlannerPath.fromChoreoTrajectory("B_Cycle").mirrorPath();
        cycle2Path = PathPlannerPath.fromChoreoTrajectory("B_Cycle2").mirrorPath();
      } else {
        cyclePath = PathPlannerPath.fromChoreoTrajectory("B_Cycle");
        cycle2Path = PathPlannerPath.fromChoreoTrajectory("B_Cycle2");
      }

      Pose2d startingPose =
          new Pose2d(cyclePath.getPoint(0).position, cyclePath.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose),
          AutoBuilder.followPath(cyclePath).alongWith(shooter.runVelocity(80)),
          Commands.waitSeconds(6.0)
              .deadlineFor(drivetrain.driveToPose(() -> getScorePose(() -> cyclePath))), // score
          shooter.runVelocity(0),
          AutoBuilder.followPath(cycle2Path).alongWith(shooter.runVelocity(80)),
          Commands.waitSeconds(6.0)
              .deadlineFor(drivetrain.driveToPose(() -> getScorePose(() -> cyclePath))), // score
          shooter.runVelocity(0),
          AutoBuilder.followPath(cycle2Path));

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
