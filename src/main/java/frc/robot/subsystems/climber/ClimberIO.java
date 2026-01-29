// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

/** Add your docs here. */
public interface ClimberIO {

    public void runVelocity(double rotationsPerSecond);

    public void runVoltage(double voltage);

    public double getMotor1StatorCurrent();

    public double getMotor2StatorCurrent();
    
    public double getMotor1Position();

    public double getMotor2Position();

    public void periodic();
}
