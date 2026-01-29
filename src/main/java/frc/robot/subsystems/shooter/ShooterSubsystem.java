package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.EagleUtil;
import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {
  private ShooterIO shooterIO;

  private double velocityGoal;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotVelocitySupplier;

  public final Trigger isAtGoalVelocity_Passing =
      new Trigger(() -> MathUtil.isNear(velocityGoal, shooterIO.getVelocity(), 10));
  public final Trigger isAtGoalVelocity_Hub =
      new Trigger(() -> MathUtil.isNear(velocityGoal, shooterIO.getVelocity(), 5));

  public ShooterSubsystem(
      CANBus rioCanbus,
      CANBus canivoreCanbus,
      StatusSignalCollection signal,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> velocity) {

    robotPoseSupplier = robotPose;
    robotVelocitySupplier = velocity;

    if (RobotBase.isSimulation()) {
      shooterIO = new ShooterIOSim();
    } else {
      shooterIO = new ShooterIOReal(rioCanbus, canivoreCanbus, signal);
    }
  }

  public Command runVelocity(double rotationsPerSecond) {
    return this.runOnce(
        () -> {
          velocityGoal = rotationsPerSecond;
          shooterIO.runVelocity(rotationsPerSecond);
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
          ChassisSpeeds robotVelocity = robotVelocitySupplier.get();
          Translation2d targetPose = EagleUtil.getRobotTarget(robotPose);
          double rotationsPerSecond = 0;
          // TODO ^ calc rps using above variables

          velocityGoal = rotationsPerSecond;
          shooterIO.runVelocity(rotationsPerSecond);
        });
  }

  @Override
  public void periodic() {
    shooterIO.periodic();
    DogLog.log("Shooter/ Current Velocity", shooterIO.getVelocity());
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
