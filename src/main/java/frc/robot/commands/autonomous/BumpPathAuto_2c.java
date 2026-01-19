package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class BumpPathAuto_2c extends SequentialCommandGroup {
  public BumpPathAuto_2c(ShooterSubsystem shooter, boolean mirror) {

    /* All your code should go inside this try-catch block */
    try {

      PathPlannerPath cyclePath = PathPlannerPath.fromChoreoTrajectory("Cycle");
      PathPlannerPath cycle2Path = PathPlannerPath.fromChoreoTrajectory("Cycle2");
      
      

      if (mirror) {
        cyclePath = cyclePath.mirrorPath();
        cycle2Path = cycle2Path.mirrorPath();
      }

      Pose2d startingPose =
          new Pose2d(
              cyclePath.getPoint(0).position, cyclePath.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(cyclePath),
          shooter.runVelocity(80),
          Commands.waitSeconds(6.0), //score
          shooter.runVelocity(0),
          AutoBuilder.followPath(cycle2Path),
          shooter.runVelocity(80),
          Commands.waitSeconds(7.0), // score
          shooter.runVelocity(0),
          AutoBuilder.followPath(cycle2Path));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
