package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.EagleUtil;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionSubsystem;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.Supplier;

public class NeutralAutos extends SequentialCommandGroup {
  public enum Routine {
    BUMP,
    TRENCH
  }

  private SwerveSubsystem drivetrain;
  private ShooterSubsystem shooter;
  private IndexerSubsystem indexer;
  private GroundIntakeLinearExtensionSubsystem groundIntakeExtend;
  private GroundIntakeRollerSubsystem groundIntakeRoller;

  public NeutralAutos(
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      GroundIntakeLinearExtensionSubsystem groundIntakeExtend,
      GroundIntakeRollerSubsystem groundIntakeRoller,
      boolean mirror,
      Routine routine,
      boolean twoCycle) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.indexer = indexer;
    this.groundIntakeRoller = groundIntakeRoller;
    this.groundIntakeExtend = groundIntakeExtend;
    try {
      // Load Paths
      PathPlannerPath cycle;
      PathPlannerPath cycletwo;
      PathPlannerPath climb;
      String pathprefix = "";

      // Set prefix for path names based on which auto
      if (routine == Routine.BUMP) {
        pathprefix = "B_";
      } else if (routine == Routine.TRENCH) {
        pathprefix = "T_";
      }

      // If on outpost side, flip the paths
      if (mirror) {
        cycle = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Cycle").mirrorPath();
        cycletwo = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Cycle2").mirrorPath();
        climb = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Climb_Mirrored");
      } else {
        cycle = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Cycle");
        cycletwo = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Cycle2");
        climb = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Climb");
      }

      Pose2d startingPose =
          new Pose2d(cycle.getPoint(0).position, cycle.getIdealStartingState().rotation());

      addCommands(
          Commands.sequence(
                  AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()).alongWith(groundIntakeExtend.homingCommand()),
                  cyclePath(cycle),
                  Commands.waitSeconds(6)
                      .deadlineFor(
                          drivetrain
                              .driveToPose(() -> getScorePose(() -> cycle))
                              .alongWith(
                                  Commands.parallel(indexer.index(), shooter.runVelocity(60)))),
                  cyclePath(cycletwo).onlyIf(() -> twoCycle),
                  climbPath(climb).andThen(Commands.idle()).onlyIf(() -> !twoCycle))
              .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }

  private Pose2d getScorePose(Supplier<PathPlannerPath> path) {
    Pose2d score;
    if (EagleUtil.isRedAlliance()) {
      score =
          new Pose2d(
              path.get().flipPath().getPoint(0).position,
              path.get().flipPath().getIdealStartingState().rotation());
    } else {
      score =
          new Pose2d(
              path.get().getPoint(0).position, path.get().getIdealStartingState().rotation());
    }
    return score;
  }

  private Command cyclePath(PathPlannerPath path) {
    return Commands.sequence(
        AutoBuilder.followPath(path)
            .deadlineFor(
                shooter.runVelocity(0),
                groundIntakeExtend.extend(),
                groundIntakeRoller.startIntake()),
        groundIntakeRoller.stopIntake());
  }

  private Command climbPath(PathPlannerPath path) {
    return Commands.sequence(
        AutoBuilder.followPath(path).deadlineFor(shooter.runVelocity(0)), Commands.idle());
  }
}
