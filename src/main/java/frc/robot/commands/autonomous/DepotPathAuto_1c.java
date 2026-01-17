package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DepotPathAuto_1c extends SequentialCommandGroup {
  public DepotPathAuto_1c() {

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

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(startingPath),
          AutoBuilder.followPath(depotPath),
          AutoBuilder.followPath(scorePath),
          Commands.waitSeconds(7.0),
          AutoBuilder.followPath(climbPath));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
