package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Trench extends SequentialCommandGroup {
  public Trench() {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      PathPlannerPath S_N = PathPlannerPath.fromChoreoTrajectory("Score_Neutral");
      PathPlannerPath Neutral = PathPlannerPath.fromChoreoTrajectory("Neutral");
      PathPlannerPath N_S = PathPlannerPath.fromChoreoTrajectory("Neutral_Score");
      PathPlannerPath Climb = PathPlannerPath.fromChoreoTrajectory("Score_Climb");
      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory("PATH NAME");

      Pose2d startingPose =
          new Pose2d(S_N.getPoint(0).position, S_N.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          Commands.sequence(
              Commands.waitSeconds(2),
              AutoBuilder.followPath(S_N),
              AutoBuilder.followPath(Neutral),
              AutoBuilder.followPath(N_S),
              Commands.waitSeconds(2),
              AutoBuilder.followPath(S_N),
              AutoBuilder.followPath(Neutral),
              AutoBuilder.followPath(N_S),
              Commands.waitSeconds(2),
              AutoBuilder.followPath(Climb))

          /*
           * TODO: The rest of the autonomous routine command
           */
          );

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
