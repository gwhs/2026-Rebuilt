package frc.robot.subsystems.shooter;

public class ShooterIODisabled implements ShooterIO {

  @Override
  public void runVelocity(double rotationsPerSecond) {}

  @Override
  public void runVelocity(double front, double back) {}

  @Override
  public void runVoltage(double voltage) {}

  @Override
  public double getVelocity() {
    return 0.0;
  }

  @Override
  public void periodic() {}
}
