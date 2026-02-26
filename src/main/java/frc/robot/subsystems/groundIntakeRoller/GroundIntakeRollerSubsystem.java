// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntakeRoller;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeRollerSubsystem extends SubsystemBase {

  private GroundIntakeRollerIO groundIntakeRollerIO;
  private double groundRollerVoltage;

  public static GroundIntakeRollerSubsystem createSim() {
    return new GroundIntakeRollerSubsystem(new GroundIntakeRollerIOSim());
  }

  public static GroundIntakeRollerSubsystem createDisabled() {
    return new GroundIntakeRollerSubsystem(new GroundIntakeRollerIODisabled());
  }

  public static GroundIntakeRollerSubsystem createReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection signal) {
    return new GroundIntakeRollerSubsystem(
        new GroundIntakeRollerIOReal(rioCanbus, canivoreCanbus, signal));
  }

  public GroundIntakeRollerSubsystem(GroundIntakeRollerIO groundIntakeRollerIO) {
    this.groundIntakeRollerIO = groundIntakeRollerIO;
  }

  public Command runVoltage(double voltage) {
    return this.runOnce(
        () -> {
          groundIntakeRollerIO.runVoltage(voltage);
          groundRollerVoltage = voltage;
        });
  }

  public Command stopIntake() {
    return runVoltage(0);
  }

  public Command startIntake() {
    return runVoltage(GroundIntakeRollerConstants.DEFAULT_INTAKE_VOLTAGE);
  }

  public Command reverseIntake() {
    return runVoltage(GroundIntakeRollerConstants.REVERSE_INTAKE_VOLTAGE);
  }

  @Override
  public void periodic() {
    groundIntakeRollerIO.periodic();
    DogLog.log("Ground Intake Roller/Goal Voltage", groundRollerVoltage);
  }

  public double getGoalRollerVoltage()
  {
    return groundRollerVoltage;
  }
}
