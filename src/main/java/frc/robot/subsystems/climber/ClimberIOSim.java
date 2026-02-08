// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.*;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {

  private DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(1), 0.03, 100),
          DCMotor.getFalcon500Foc(1),
          0.1,
          0.1);

  private double targetPosition = 0; // rotation

  @Override
  public void runVoltage(double voltage) {
    motor.setInputVoltage(voltage);
  }

  public void runVoltage(double voltage, boolean ignoreSoftwareLimit) {
    motor.setInputVoltage(voltage);
  }

  @Override
  public void periodic() {
    motor.update(.020);
    double error = targetPosition - getMotor1Position();
    double kP = 20;
    double voltage = kP * error;
    motor.setInputVoltage(voltage);
  }

  public double getMotor1Position() {
    return motor.getAngularPositionRotations();
  }

  public void setPosition(double rotation) {}

  public void runPosition(double rotation) {
    targetPosition = rotation;
  }

  public boolean getReverseLimitSwitch() {
    return getMotor1Position() >= 0.05;
  }
}
