package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Trench_2c extends SequentialCommandGroup {
  public Trench_2c(boolean Mirror) {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      PathPlannerPath Neutral = PathPlannerPath.fromChoreoTrajectory("Neutral");
      if (Mirror) {
        Neutral = Neutral.mirrorPath();
      }

      Pose2d startingPose =
          new Pose2d(Neutral.getPoint(0).position, Neutral.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(Neutral),
          Commands.waitSeconds(6),
          AutoBuilder.followPath(Neutral),
          Commands.waitSeconds(6)
          /*
           * TODO: The rest of the autonomous routine command
           */
          );

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
