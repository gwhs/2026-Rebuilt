package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.EagleUtil;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DepotOutpost extends SequentialCommandGroup {
  public DepotOutpost(
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      GroundIntakeRollerSubsystem groundIntakeRoller) {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      PathPlannerPath startingPath = PathPlannerPath.fromChoreoTrajectory("D_Start_Outpost");
      PathPlannerPath scorePath = PathPlannerPath.fromChoreoTrajectory("D_Outpost_Tower");

      Pose2d startingPose =
          new Pose2d(
              startingPath.getPoint(0).position, startingPath.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(startingPath).deadlineFor(groundIntakeRoller.startIntake()),
          Commands.waitSeconds(3),
          AutoBuilder.followPath(scorePath)
              .alongWith(
                  Commands.sequence(
                      Commands.waitSeconds(1.8),
                      Commands.parallel(
                          groundIntakeRoller.stopIntake(),
                          indexer.index(),
                          shooter.cruiseControl(),
                          EagleUtil.shootInSim(drivetrain)
                              .onlyIf(() -> RobotBase.isSimulation())))));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
