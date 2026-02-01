// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

/** Add your docs here. */
public interface ClimberIO {

    public void runVoltage(double voltage);

    public void runVoltage(double voltage, boolean ignoreSoftwareLimit);
    
    public double getMotor1Position();

    public void periodic();

    public void setPosition(double rotation);

    public void runPosition(double rotation);

    public boolean getReverseLimitSwitch();
}
