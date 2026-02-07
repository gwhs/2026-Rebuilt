package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.EagleUtil;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionSubsystem;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DepotPathAuto_1c extends SequentialCommandGroup {
  public DepotPathAuto_1c(
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      GroundIntakeLinearExtensionSubsystem groundIntakeExtend,
      GroundIntakeRollerSubsystem groundIntakeRoller,
      ClimberSubsystem climber) {

    try {

      PathPlannerPath startingPath = PathPlannerPath.fromChoreoTrajectory("D_Start_Depot");
      PathPlannerPath climbPath = PathPlannerPath.fromChoreoTrajectory("D_Depot_Climb");

      Pose2d startingPose =
          new Pose2d(
              startingPath.getPoint(0).position, startingPath.getIdealStartingState().rotation());
      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          Commands.parallel(
              AutoBuilder.followPath(startingPath)
                  .deadlineFor(climber.homingCommand())
                  .deadlineFor(
                      Commands.sequence(
                          Commands.waitSeconds(0.5),
                          Commands.parallel(
                              groundIntakeExtend.extend(), groundIntakeRoller.startIntake()),
                          Commands.waitSeconds(2),
                          shooter.cruiseControl()))),
          AutoBuilder.followPath(climbPath).deadlineFor(shooter.cruiseControl()),
          Commands.waitSeconds(6)
              .deadlineFor(
                  shooter.cruiseControl(),
                  EagleUtil.shootInSim(drivetrain).onlyIf(() -> RobotBase.isSimulation())),
          climber.runPosition(ClimberConstants.CLIMB).alongWith(shooter.runVoltage(0)));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }

  private Pose2d getScorePose(PathPlannerPath path) {
    Pose2d score;
    if (EagleUtil.isRedAlliance()) {
      score =
          new Pose2d(
              path.flipPath().getPoint(0).position,
              path.flipPath().getIdealStartingState().rotation());
    } else {
      score = new Pose2d(path.getPoint(0).position, path.getIdealStartingState().rotation());
    }
    return score;
  }
}
