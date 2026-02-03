package frc.robot.subsystems.groundIntakeLinearExtension;

public class GroundIntakeLinearExtensionIOSim implements GroundIntakeLinearExtensionIO {

  public void periodic() {}

  @Override
  public void runVoltage(double voltage) {}

  public void runVoltage(double volts, boolean ignoreSoftwareLimit) {}

  public void runPosition(double rotation) {}

  public double getRotation() {
    return 0;
  }

  public void setPosition(double newValue) {}

  public double getStatorCurrent() {
    return 0;
  }

  public boolean getReverseLimit() {
    return false;
  }

  public boolean getForwardLimit() {
    return false;
  }
}
