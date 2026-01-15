package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BumpPathAuto extends SequentialCommandGroup {
  public BumpPathAuto() {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      PathPlannerPath cyclePath = PathPlannerPath.fromChoreoTrajectory("Cycle");
      PathPlannerPath climbPath = PathPlannerPath.fromChoreoTrajectory("Climb");

      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory("PATH NAME");

      Pose2d startingPose =
          new Pose2d(cyclePath.getPoint(0).position, cyclePath.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(cyclePath),
          // stow/protect ground intake

          Commands.waitSeconds(3.0), // score
          AutoBuilder.followPath(cyclePath),
          /*
          Commands.parallel(
              AutoBuilder.followPath(cross2Path),
              Commands.sequence(
                  Commands.waitSeconds(0.8)
                  // deploy ground intake
                  )),
          */
          AutoBuilder.followPath(climbPath),
          // stow/protect ground intake
          Commands.waitSeconds(3.0)
          /*
           * TODO: The rest of the autonomous routine command
           */
          );

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
