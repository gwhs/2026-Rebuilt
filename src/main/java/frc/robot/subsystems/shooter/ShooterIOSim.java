package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

  private FlywheelSim motor =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1),
          DCMotor.getKrakenX60Foc(2));

  private double targetVelocity = 0; // rotationsPerSecond

  @Override
  public void runVelocity(double rotationsPerSecond) {
    targetVelocity = rotationsPerSecond;
  }

  @Override
  public void runVoltage(double voltage) {
    motor.setInputVoltage(voltage);
  }

  @Override
  public double getVelocity() {
    return motor.getAngularVelocity().in(RotationsPerSecond);
  }

  @Override
  public void periodic() {
    motor.update(.020);
    double error = targetVelocity - getVelocity();
    double kP = 50;
    double voltage = kP * error;
    motor.setInputVoltage(voltage);
  }
}
