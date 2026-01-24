// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class IndexerIOReal implements IndexerIO {

  private final TalonFX motor1;
  private final TalonFX motor2;

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<AngularVelocity> motor1Velocity;
  private final StatusSignal<AngularAcceleration> motor1Acceleration;
  private final StatusSignal<Temperature> motor1Temp;
  private final StatusSignal<Double> motor1ClosedLoopGoal;

  private final StatusSignal<Voltage> motor2Voltage;
  private final StatusSignal<Current> motor2StatorCurrent;
  private final StatusSignal<AngularVelocity> motor2Velocity;
  private final StatusSignal<AngularAcceleration> motor2Acceleration;
  private final StatusSignal<Temperature> motor2Temp;
  private final StatusSignal<Double> motor2ClosedLoopGoal;

  private final Alert motor1NotConnectedAlert =
      new Alert("Indexer Motor 1 Not Connected ", AlertType.kError);

  private final Alert motor2NotConnectedAlert =
      new Alert("Indexer Motor 2 Not Connected ", AlertType.kError);

  public IndexerIOReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection statusSignalCollection) {

    motor1 = new TalonFX(IndexerConstants.MOTOR_1_ID, rioCanbus);
    motor2 = new TalonFX(IndexerConstants.MOTOR_2_ID, rioCanbus);

    motor1Voltage = motor1.getMotorVoltage();
    motor1StatorCurrent = motor1.getStatorCurrent();
    motor1Velocity = motor1.getVelocity();
    motor1Temp = motor1.getDeviceTemp();
    motor1Acceleration = motor1.getAcceleration();
    motor1ClosedLoopGoal = motor1.getClosedLoopReference();

    motor2Voltage = motor2.getMotorVoltage();
    motor2StatorCurrent = motor2.getStatorCurrent();
    motor2Velocity = motor2.getVelocity();
    motor2Temp = motor2.getDeviceTemp();
    motor2Acceleration = motor2.getAcceleration();
    motor2ClosedLoopGoal = motor2.getClosedLoopReference();

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 40;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Slot0.kS = 0.125;
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.065;
    talonFXConfig.Slot0.kP = 0.5;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i <= 5; i++) {
      status = motor1.getConfigurator().apply(talonFXConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      new Alert("Indexer: Could not configure Motor 1. Error" + status.toString(), AlertType.kError)
          .set(true);
    }

    talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i <= 5; i++) {
      status = motor2.getConfigurator().apply(talonFXConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      new Alert("Indexer: Could not configure Motor 2. Error" + status.toString(), AlertType.kError)
          .set(true);
    }

    statusSignalCollection.addSignals(
        motor1Voltage,
        motor1StatorCurrent,
        motor1Velocity,
        motor1Temp,
        motor1Acceleration,
        motor1ClosedLoopGoal,
        motor2Voltage,
        motor2StatorCurrent,
        motor2Velocity,
        motor2Temp,
        motor2Acceleration,
        motor2ClosedLoopGoal);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor1Voltage,
        motor1StatorCurrent,
        motor1Velocity,
        motor1Temp,
        motor1Acceleration,
        motor1ClosedLoopGoal,
        motor2Voltage,
        motor2StatorCurrent,
        motor2Velocity,
        motor2Temp,
        motor2Acceleration,
        motor2ClosedLoopGoal);
  }

  public void periodic() {
    DogLog.log("Indexer/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Indexer/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Indexer/Motor 1 Velocity", motor1Velocity.getValueAsDouble());
    DogLog.log("Indexer/Motor 1 Temperature", motor1Temp.getValueAsDouble());
    DogLog.log("Indexer/Motor 1 Acceleration", motor1Acceleration.getValueAsDouble());
    DogLog.log("Indexer/Motor 1 Closed Loop Goal", motor1ClosedLoopGoal.getValueAsDouble());
    DogLog.log("Indexer/Motor 2 Voltage", motor2Voltage.getValueAsDouble());
    DogLog.log("Indexer/Motor 2 Stator Current", motor2StatorCurrent.getValueAsDouble());
    DogLog.log("Indexer/Motor 2 Velocity", motor2Velocity.getValueAsDouble());
    DogLog.log("Indexer/Motor 2 Temperature", motor2Temp.getValueAsDouble());
    DogLog.log("Indexer/Motor 2 Acceleration", motor2Acceleration.getValueAsDouble());
    DogLog.log("Indexer/Motor 2 Closed Loop Goal", motor2ClosedLoopGoal.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
    motor2NotConnectedAlert.set(!motor2.isConnected());
  }

  public void runVoltage(double voltage) {
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }
}
