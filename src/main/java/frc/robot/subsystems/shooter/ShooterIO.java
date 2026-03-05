package frc.robot.subsystems.shooter;

public interface ShooterIO {
  public void runVelocity(double rotationsPerSecond);

  public void runVelocity(double front, double back);

  public void runVoltage(double voltage);

  public double getVelocity();

  public void periodic();
}
