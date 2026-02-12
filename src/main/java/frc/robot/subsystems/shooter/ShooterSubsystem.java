package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.EagleUtil;
import frc.robot.ShotCalculator;
import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {

  public static ShooterSubsystem createSim(
      Supplier<Pose2d> robotPose, Supplier<Pose2d> robotTarget) {
    return new ShooterSubsystem(new ShooterIOSim(), robotPose, robotTarget);
  }

  public static ShooterSubsystem createDisabled(
      Supplier<Pose2d> robotPose, Supplier<Pose2d> robotTarget) {
    return new ShooterSubsystem(new ShooterIODisabled(), robotPose, robotTarget);
  }

  public static ShooterSubsystem createReal(
      CANBus rioCanbus,
      CANBus canivoreCanbus,
      StatusSignalCollection signal,
      Supplier<Pose2d> robotPose,
      Supplier<Pose2d> robotTarget) {
    return new ShooterSubsystem(
        new ShooterIOReal(rioCanbus, canivoreCanbus, signal), robotPose, robotTarget);
  }

  private ShooterIO shooterIO;

  private double velocityGoal;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<Pose2d> robotTargetSupplier;

  public final Trigger isAtGoalVelocity_Passing =
      new Trigger(() -> MathUtil.isNear(velocityGoal, shooterIO.getVelocity(), 10));
  public final Trigger isAtGoalVelocity_Hub =
      new Trigger(() -> MathUtil.isNear(velocityGoal, shooterIO.getVelocity(), 5));

  public ShooterSubsystem(
      ShooterIO shooterIO, Supplier<Pose2d> robotPose, Supplier<Pose2d> robotTarget) {
    this.shooterIO = shooterIO;
    robotTargetSupplier = robotTarget;
    robotPoseSupplier = robotPose;
  }

  public Command runVelocity(double rotationsPerSecond) {
    return this.run(
        () -> {
          runShooterWithClamp(rotationsPerSecond);
        });
  }

  public Command runVoltage(double voltage) {
    return this.runOnce(
        () -> {
          shooterIO.runVoltage(voltage);
        });
  }

  public Command cruiseControl() {
    return this.run(
        () -> {
          Pose2d robotPose = robotPoseSupplier.get();
          Pose2d targetPose = robotTargetSupplier.get();
          double robotTargetDist = EagleUtil.getRobotTargetDistance(robotPose, targetPose);
          double rotationsPerSecond = ShotCalculator.getShootVelocity(robotTargetDist);

          runShooterWithClamp(rotationsPerSecond);
        });
  }

  public void runShooterWithClamp(double rps) {
    double clampedRps =
              Math.max(
                  ShooterConstants.MIN_RPS, Math.min(ShooterConstants.MAX_RPS, rps));
          velocityGoal = clampedRps;

          if (shooterIO.getVelocity() <= velocityGoal - ShooterConstants.VELOCITY_TOLERANCE) {
            shooterIO.runVoltage(12);
          } else {
            shooterIO.runVelocity(clampedRps);
          }
  }

  public Command preSpin() {
    return this.run(
        () -> {
          Pose2d robotPose = robotPoseSupplier.get();
          Pose2d targetPose = robotTargetSupplier.get();
          double robotTargetDist = EagleUtil.getRobotTargetDistance(robotPose, targetPose);
          double rotationsPerSecond = ShotCalculator.getShootVelocity(robotTargetDist);
          runVoltage(0);

          // does not actually pre-spin
        });
  }

  @Override
  public void periodic() {
    shooterIO.periodic();
    DogLog.log("Shooter/Current Velocity", shooterIO.getVelocity());
    DogLog.log("Shooter/Goal Velocity", velocityGoal);
    DogLog.log("Shooter/At Goal Velocity Hub", this.isAtGoalVelocity_Hub.getAsBoolean());
    DogLog.log("Shooter/At Goal Velocity Passing", this.isAtGoalVelocity_Passing.getAsBoolean());
  }

  /**
   * @return the shooter's rps
   */
  public double getVelocity() {
    return shooterIO.getVelocity();
  }
}
