package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Trench_1c extends SequentialCommandGroup {
  public Trench_1c(boolean mirror) {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      PathPlannerPath neutral = PathPlannerPath.fromChoreoTrajectory("Neutral");
      PathPlannerPath climb = PathPlannerPath.fromChoreoTrajectory("Score_Climb");

      if (mirror) {
        neutral = neutral.mirrorPath();
        climb = PathPlannerPath.fromChoreoTrajectory("Score_Climb_Mirror");
      }
      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory("PATH NAME");

      Pose2d startingPose =
          new Pose2d(neutral.getPoint(0).position, neutral.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(neutral),
          Commands.waitSeconds(6),
          AutoBuilder.followPath(climb)
          /*
           * TODO: The rest of the autonomous routine command
           */
          );

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
