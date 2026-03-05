package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public interface ShooterIO {
  public void runVelocity(double rotationsPerSecond);

  public void runVelocity(double front, double back);

  public void runVoltage(double voltage);

  public double getVelocity();

  public void periodic();


}
