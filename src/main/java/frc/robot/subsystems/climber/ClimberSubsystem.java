// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private ClimberIO climberIO;
  private double climberVoltage;
  private double velocityGoal;

  public ClimberSubsystem(CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection signal) {
    if (RobotBase.isSimulation()) {
      climberIO = new ClimberIOSim();
    } else {
      climberIO = new ClimberIOReal(rioCanbus, canivoreCanbus, signal);
    }
  }

  public Command runVoltage(double voltage) {
    return this.runOnce(
        () -> {
          climberIO.runVoltage(voltage);
          climberVoltage = voltage;
        });
  }

  public Command runVelocity(double rotationsPerSecond) {
    return this.runOnce(
        () -> {
          velocityGoal = rotationsPerSecond;
          climberIO.runVelocity(rotationsPerSecond);
        });
  }

  @Override
  public void periodic() {
    climberIO.periodic();
    DogLog.log("Climber/Goal Voltage", climberVoltage);
  }

  public double getMotor1Position() {
    return climberIO.getMotor1Position();
  }

  public double getMotor2Position() {
    return climberIO.getMotor2Position();
  }

  public double getMotor1StatorCurrent() {
    return climberIO.getMotor1StatorCurrent();
  }

  public double getMotor2StatorCurrent() {
    return climberIO.getMotor2StatorCurrent();
  }
}
