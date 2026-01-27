package frc.robot.subsystems.groundIntakeLinearExtension;

public interface GroundIntakeLinearExtensionIO {
  // public void setAngle(double angle);

  // public double getPosition();

  public void periodic();

  public void runVoltage(double volts);

  public void runPosition(double rotation);

  public double getRotation();
}
