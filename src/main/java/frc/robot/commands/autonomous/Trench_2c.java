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

public class Trench_2c extends SequentialCommandGroup {
  public Trench_2c(SwerveSubsystem drivetrain, ShooterSubsystem shooter, boolean mirror) {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      PathPlannerPath neutral = PathPlannerPath.fromChoreoTrajectory("T_Neutral");
      PathPlannerPath score_neutral = PathPlannerPath.fromChoreoTrajectory("T_Score_Neutral");
      if (mirror) {
        neutral = neutral.mirrorPath();
        score_neutral = score_neutral.mirrorPath();
      }

      Pose2d startingPose =
          new Pose2d(neutral.getPoint(0).position, neutral.getIdealStartingState().rotation());
      Pose2d score;
      if (!EagleUtil.isRedAlliance()) {
        score =
            new Pose2d(
                neutral.flipPath().getPoint(0).position,
                neutral.flipPath().getIdealStartingState().rotation());
      } else {
        score =
            new Pose2d(neutral.getPoint(0).position, neutral.getIdealStartingState().rotation());
      }

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(neutral).deadlineFor(shooter.runVelocity(80)),
          Commands.waitSeconds(6).deadlineFor(drivetrain.driveToPose(() -> score)),
          AutoBuilder.followPath(neutral).deadlineFor(shooter.runVelocity(80)),
          Commands.waitSeconds(6).deadlineFor(drivetrain.driveToPose(() -> score)),
          AutoBuilder.followPath(score_neutral)
          /*
           * TODO: The rest of the autonomous routine command
           */
          );

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
