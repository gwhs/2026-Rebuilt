// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO{

    @Override
  public void runVoltage(double voltage) {}

    @Override
  public void runVelocity(double rotationsPerSecond) {}

   @Override
  public void periodic() {}

   @Override
  public double getMotor1Position() {
    return 0;
  }

   @Override
  public double getMotor2Position() {
    return 0;
  }

   @Override
  public double getMotor1StatorCurrent() {
    return 0;
  }

   @Override
  public double getMotor2StatorCurrent() {
      return 0;
    }
}
