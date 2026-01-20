package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShooterSubsystem extends SubsystemBase {

  public static ShooterSubsystem createSim() {
    return new ShooterSubsystem(new ShooterIOSim());
  }

  public static ShooterSubsystem createDisabled() {
    return new ShooterSubsystem(new ShooterIODisabled());
  }

  public static ShooterSubsystem createReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection signal) {
    return new ShooterSubsystem(new ShooterIOReal(rioCanbus, canivoreCanbus, signal));
  }

  private ShooterIO shooterIO;

  private double velocityGoal;

  public final Trigger isAtGoalVelocity_Passing =
      new Trigger(() -> MathUtil.isNear(velocityGoal, shooterIO.getVelocity(), 10));
  public final Trigger isAtGoalVelocity_Hub =
      new Trigger(() -> MathUtil.isNear(velocityGoal, shooterIO.getVelocity(), 5));

  public ShooterSubsystem(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
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

  @Override
  public void periodic() {
    shooterIO.periodic();
    DogLog.log("Shooter/ Current Velocity", shooterIO.getVelocity());
    DogLog.log("Shooter/Goal Velocity", velocityGoal);
  }

  /**
   * @return the climb's position
   */
  public double getVelocity() {
    return shooterIO.getVelocity();
  }
}
