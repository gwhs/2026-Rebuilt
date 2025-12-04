package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class Template extends PathPlannerAuto {
  public Template(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    /* All your code should go inside this try-catch block */
    try {

      PathPlannerPath S3Leave = PathPlannerPath.fromPathFile("S3-C5");

      Pose2d startingPose =
          new Pose2d(S3Leave.getPoint(0).position, S3Leave.getIdealStartingState().rotation());

      isRunning().onTrue(Commands.sequence(AutoBuilder.resetOdom(startingPose)));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
