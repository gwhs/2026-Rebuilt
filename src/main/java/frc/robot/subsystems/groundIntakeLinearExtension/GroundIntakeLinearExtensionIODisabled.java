// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntakeLinearExtension;

/** Add your docs here. */
public class GroundIntakeLinearExtensionIODisabled implements GroundIntakeLinearExtensionIO{

  @Override
  public void periodic() {}

  @Override
  public void runVoltage(double volts) {}

  @Override
  public void runPosition(double rotation) {}

  @Override
  public double getRotation() {
    return 0.0;
  }

  @Override
  public void setPosition(double newValue) {}

  @Override
  public void runVoltage(double volts, boolean ignoreSoftwareLimit) {}

  @Override
  public double getStatorCurrent() {
    return 0.0;
  }

  @Override
  public boolean getReverseLimit() {
    return false;
  }

  @Override
  public boolean getForwardLimit() {
    return false;
  }
}
