package frc.robot.subsystems.groundIntakeLinearExtension;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class GroundIntakeLinearExtensionIOSim implements GroundIntakeLinearExtensionIO {

  private DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.001, 5),
          DCMotor.getFalcon500Foc(1),
          0.1,
          0.1);

  private double targetPosition = 0;

  public void periodic() {
    motor.update(.020);
    double error = targetPosition - getRotation();
    double kP = 3;
    double voltage = kP * error;
    motor.setInputVoltage(voltage);
  }

  @Override
  public void runVoltage(double voltage) {
    motor.setInputVoltage(voltage);
  }

  public void runVoltage(double volts, boolean ignoreSoftwareLimit) {
    motor.setInputVoltage(volts);
  }

  public void runPosition(double rotation) {
    targetPosition = rotation;
  }

  public double getRotation() {
    return motor.getAngularPositionRotations();
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
