package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.EagleUtil;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Preload extends SequentialCommandGroup {
  public Preload(SwerveSubsystem drivetrain, ShooterSubsystem shooter, IndexerSubsystem indexer) {

    try {

      Pose2d startingPose = new Pose2d(3.57, 3.41, new Rotation2d(0.53));

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          Commands.parallel(
              indexer.index(),
              shooter.cruiseControl(),
              EagleUtil.shootInSim(drivetrain).onlyIf(() -> RobotBase.isSimulation())));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
