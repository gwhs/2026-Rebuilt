// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntakeRoller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class GroundIntakeRollerIOReal implements GroundIntakeRollerIO {

  private final TalonFX motor1;
  private final TalonFX motor2;

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<Temperature> motor1Temp;

  private final StatusSignal<Voltage> motor2Voltage;
  private final StatusSignal<Current> motor2StatorCurrent;
  private final StatusSignal<Temperature> motor2Temp;

  private final Follower controlRequest =
      new Follower(GroundIntakeRollerConstants.MOTOR_1_ID, MotorAlignmentValue.Opposed);

  private final Alert motor1NotConnectedAlert =
      new Alert("Ground Intake Roller Motor 1 Not Connected ", AlertType.kError);

  private final Alert motor2NotConnectedAlert =
      new Alert("Ground Intake Roller Motor 2 Not Connected ", AlertType.kError);

  @SuppressWarnings("resource")
  public GroundIntakeRollerIOReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection signal) {

    motor1 = new TalonFX(GroundIntakeRollerConstants.MOTOR_1_ID, rioCanbus);
    motor2 = new TalonFX(GroundIntakeRollerConstants.MOTOR_2_ID, rioCanbus);

    motor1Voltage = motor1.getMotorVoltage();
    motor1StatorCurrent = motor1.getStatorCurrent();
    motor1Temp = motor1.getDeviceTemp();

    motor2Voltage = motor2.getMotorVoltage();
    motor2StatorCurrent = motor2.getStatorCurrent();
    motor2Temp = motor2.getDeviceTemp();

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 95;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    StatusCode status = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i <= 5; i++) {
      status = motor1.getConfigurator().apply(talonFXConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      new Alert(
              "Ground Roller: Could not configure Motor 1. Error" + status.toString(),
              AlertType.kError)
          .set(true);
    }

    for (int i = 0; i <= 5; i++) {
      status = motor2.getConfigurator().apply(talonFXConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      new Alert(
              "Ground Roller: Could not configure Motor 2. Error" + status.toString(),
              AlertType.kError)
          .set(true);
    }

    signal.addSignals(
        motor1Voltage,
        motor1StatorCurrent,
        motor1Temp,
        motor2Voltage,
        motor2StatorCurrent,
        motor2Temp);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor1Voltage,
        motor1StatorCurrent,
        motor1Temp,
        motor2Voltage,
        motor2StatorCurrent,
        motor2Temp);
  }

  public void runVoltage(double voltage) {
    motor1.setVoltage(voltage);
    motor2.setControl(controlRequest);
  }

  public void periodic() {
    DogLog.log("Ground Roller/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Ground Roller/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Ground Roller/Motor 1 Temperature", motor1Temp.getValueAsDouble());
    DogLog.log("Ground Roller/Motor 2 Voltage", motor2Voltage.getValueAsDouble());
    DogLog.log("Ground Roller/Motor 2 Stator Current", motor2StatorCurrent.getValueAsDouble());
    DogLog.log("Ground Roller/Motor 2 Temperature", motor2Temp.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
    motor2NotConnectedAlert.set(!motor2.isConnected());
  }
}
