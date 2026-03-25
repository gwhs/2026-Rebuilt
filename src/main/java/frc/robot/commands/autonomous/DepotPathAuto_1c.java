package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.EagleUtil;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionSubsystem;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DepotPathAuto_1c extends SequentialCommandGroup {
  public DepotPathAuto_1c(
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      GroundIntakeLinearExtensionSubsystem groundIntakeExtend,
      GroundIntakeRollerSubsystem groundIntakeRoller,
      ClimberSubsystem climber) {

    try {

      PathPlannerPath startingPath = PathPlannerPath.fromChoreoTrajectory("D_Start_Depot");
      PathPlannerPath climbPath = PathPlannerPath.fromChoreoTrajectory("D_Depot_Climb");
      PathPlannerPath neutralPath = PathPlannerPath.fromChoreoTrajectory("D_Start_Bump");

      Pose2d startingPose =
          new Pose2d(
              startingPath.getPoint(0).position, startingPath.getIdealStartingState().rotation());
      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          Commands.waitSeconds(1)
              .deadlineFor(
                  shooter.cruiseControl(),
                  indexer.index(),
                  groundIntakeExtend.retract(),
                  EagleUtil.shootInSim(drivetrain).onlyIf(() -> RobotBase.isSimulation())),
          Commands.parallel(
              AutoBuilder.followPath(startingPath)
                  .deadlineFor(
                      Commands.sequence(
                          Commands.parallel(
                              climber.homingCommand().onlyIf(() -> RobotBase.isReal()),
                              groundIntakeExtend.homingCommand().onlyIf(() -> RobotBase.isReal())),
                          Commands.parallel(
                              groundIntakeExtend.extend(),
                              groundIntakeRoller.startIntake(),
                              shooter.stopShooter()),
                          Commands.waitSeconds(2),
                          shooter.preSpin()))),
          AutoBuilder.followPath(climbPath),
          Commands.waitSeconds(5)
              .deadlineFor(
                  shooter.cruiseControl(),
                  indexer.index(),
                  groundIntakeExtend.retract(),
                  EagleUtil.shootInSim(drivetrain).onlyIf(() -> RobotBase.isSimulation())),
          AutoBuilder.followPath(neutralPath)
              .deadlineFor(
                  shooter.stopShooter(),
                  indexer.runVoltage(0),
                  groundIntakeExtend.extend(),
                  groundIntakeRoller.startIntake()));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
