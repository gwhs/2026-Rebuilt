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
      PathPlannerPath startingPath = PathPlannerPath.fromChoreoTrajectory("Unload");
      PathPlannerPath cross1Path = PathPlannerPath.fromChoreoTrajectory("Cross1");
      PathPlannerPath gather1Path = PathPlannerPath.fromChoreoTrajectory("Gather1");
      PathPlannerPath return1Path = PathPlannerPath.fromChoreoTrajectory("Return1");
      PathPlannerPath score1Path = PathPlannerPath.fromChoreoTrajectory("Score1");
      PathPlannerPath cross2Path = PathPlannerPath.fromChoreoTrajectory("Cross2");
      PathPlannerPath gather2Path = PathPlannerPath.fromChoreoTrajectory("Gather2");
      PathPlannerPath return2Path = PathPlannerPath.fromChoreoTrajectory("Return2");
      PathPlannerPath score2Path = PathPlannerPath.fromChoreoTrajectory("Score2");

      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory("PATH NAME");

      Pose2d startingPose =
          new Pose2d(
              startingPath.getPoint(0).position, startingPath.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          Commands.waitSeconds(3), // score preloaded fuel
          Commands.parallel(
              AutoBuilder.followPath(cross1Path),
              Commands.sequence(
                  Commands.waitSeconds(0.8)
                  // deploy ground intake
                  )),
          AutoBuilder.followPath(gather1Path),
          // stow/protect ground intake
          AutoBuilder.followPath(return1Path),
          Commands.waitSeconds(3.0), // score
          AutoBuilder.followPath(score1Path),
          Commands.parallel(
              AutoBuilder.followPath(cross2Path),
              Commands.sequence(
                  Commands.waitSeconds(0.8)
                  // deploy ground intake
                  )),
          AutoBuilder.followPath(gather2Path),
          // stow/protect ground intake
          AutoBuilder.followPath(return2Path),
          AutoBuilder.followPath(score2Path),
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
