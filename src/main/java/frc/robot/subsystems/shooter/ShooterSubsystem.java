package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShooterSubsystem extends SubsystemBase {
  private ShooterIO shooterIO;

  private double velocityGoal;

  public final Trigger isAtGoalVelocity_Passing =
      new Trigger(() -> MathUtil.isNear(velocityGoal, shooterIO.getVelocity(), 10));
  public final Trigger isAtGoalVelocity_Hub =
      new Trigger(() -> MathUtil.isNear(velocityGoal, shooterIO.getVelocity(), 5));

  public ShooterSubsystem(CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection signal) {

    if (RobotBase.isSimulation()) {
      shooterIO = new ShooterIOSim();
    } else {
      shooterIO = new ShooterIOReal(rioCanbus, canivoreCanbus, signal);
    }

    SmartDashboard.putData("3V", runVoltage(3));
    SmartDashboard.putData("10V", runVoltage(10));
    SmartDashboard.putData("-3V", runVoltage(-3));
    SmartDashboard.putData("-10V", runVoltage(-10));
    SmartDashboard.putData("50 RPS", runVelocity(50));
    SmartDashboard.putData("100 RPS", runVelocity(100));
    SmartDashboard.putData("-50 RPS", runVelocity(-50));
    SmartDashboard.putData("-100 RPS", runVelocity(-100));
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
