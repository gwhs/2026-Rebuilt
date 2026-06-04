// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.blocker;

public class BlockerIODisabled implements BlockerIO {

  @Override
  public void runPosition(double rotation) {}

  @Override
  public void periodic() {}

  @Override
  public double getMotor1Position() {
    return 0.0;
  }
}
