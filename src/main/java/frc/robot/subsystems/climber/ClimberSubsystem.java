// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private ClimberIO climberIO;
  private double goalRotation;

  public static ClimberSubsystem createSim() {
    return new ClimberSubsystem(new ClimberIOSim());
  }

  public static ClimberSubsystem createDisabled() {
    return new ClimberSubsystem(new ClimberIODisabled());
  }

  public static ClimberSubsystem createReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection signal) {
    return new ClimberSubsystem(new ClimberIOReal(rioCanbus, canivoreCanbus, signal));
  }

  public ClimberSubsystem(ClimberIO climberIO) {
    this.climberIO = climberIO;
    SmartDashboard.putData("Climber Homing Command", homingCommand());
  }

  public Command runVoltage(double voltage) {
    return this.runOnce(
        () -> {
          climberIO.runVoltage(voltage);
        });
  }

  public Command runPosition(double rotation) {
    return this.runOnce(
            () -> {
              double rotationClamp =
                  MathUtil.clamp(
                      rotation, ClimberConstants.MIN_ROTATION, ClimberConstants.MAX_ROTATION);
              climberIO.runPosition(rotationClamp);
              goalRotation = rotationClamp;
            })
        .andThen(
            Commands.waitUntil(
                () -> MathUtil.isNear(goalRotation, climberIO.getMotor1Position(), 0.1)));
  }

  public Command homingCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> climberIO.runVoltage(-3, true)),
        Commands.waitUntil(() -> climberIO.getReverseLimitSwitch()),
        Commands.runOnce(() -> climberIO.runVoltage(0)),
        Commands.runOnce(() -> climberIO.setPosition(0))).onlyIf(() -> RobotBase.isReal());
  }

  @Override
  public void periodic() {
    climberIO.periodic();
    DogLog.log("Climber/Goal Rotation", goalRotation);
    DogLog.log("Climber/Current Position", getMotor1Position());
  }

  public double getMotor1Position() {
    return climberIO.getMotor1Position();
  }
}
