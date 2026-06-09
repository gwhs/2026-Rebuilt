// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.blocker;

public interface BlockerIO {
  public double getMotor1Position();

  public void runPosition(double angle);

  public void periodic();
}
