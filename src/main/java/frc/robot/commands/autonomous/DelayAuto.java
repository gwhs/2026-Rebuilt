package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionSubsystem;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerSubsystem;

public class DelayAuto extends SequentialCommandGroup {
  public DelayAuto(
      GroundIntakeLinearExtensionSubsystem groundIntakeExtend,
      GroundIntakeRollerSubsystem groundIntakeRoller) {

    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("D_Start_Bump");
      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory("PATH NAME");

      Pose2d startingPose =
          new Pose2d(path.getPoint(0).position, path.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          Commands.waitSeconds(14)
              .deadlineFor(groundIntakeExtend.homingCommand().onlyIf(() -> RobotBase.isReal())),
          Commands.parallel(
              AutoBuilder.followPath(path),
              Commands.sequence(
                  Commands.waitSeconds(2.5),
                  Commands.parallel(
                      groundIntakeExtend.extend2(), groundIntakeRoller.startIntake()))));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
