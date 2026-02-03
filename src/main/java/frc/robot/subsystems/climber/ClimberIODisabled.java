// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

/** Add your docs here. */
public class ClimberIODisabled implements ClimberIO {

  @Override
  public void runPosition(double rotation) {}

  @Override
  public void runVoltage(double voltage, boolean ignoreSoftwareLimit) {}

  @Override
  public void runVoltage(double voltage) {}

  @Override
  public void setPosition(double rotation) {}

  @Override
  public double getMotor1Position() {
    return 0.0;
  }

  @Override
  public boolean getReverseLimitSwitch() {
    return false;
  }

  @Override
  public void periodic() {}
}
