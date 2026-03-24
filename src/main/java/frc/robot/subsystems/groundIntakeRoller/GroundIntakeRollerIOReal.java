// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntakeRoller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class GroundIntakeRollerIOReal implements GroundIntakeRollerIO {

  private final TalonFXS motor1;

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<Temperature> motor1Temp;

  private final Alert motor1NotConnectedAlert =
      new Alert("Ground Intake Roller Motor 1 Not Connected ", AlertType.kError);

  @SuppressWarnings("resource")
  public GroundIntakeRollerIOReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection signal) {

    motor1 = new TalonFXS(GroundIntakeRollerConstants.MOTOR_1_ID, rioCanbus);

    motor1Voltage = motor1.getMotorVoltage();
    motor1StatorCurrent = motor1.getStatorCurrent();
    motor1Temp = motor1.getDeviceTemp();

    TalonFXSConfiguration talonFXConfig = new TalonFXSConfiguration();

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 95;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    talonFXConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;

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

    status = StatusCode.StatusCodeNotInitialized;

    signal.addSignals(
        motor1Voltage,
        motor1StatorCurrent,
        motor1Temp);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor1Voltage,
        motor1StatorCurrent,
        motor1Temp);
  }

  public void runVoltage(double voltage) {
    motor1.setVoltage(voltage);
  }

  public void periodic() {
    DogLog.log("Ground Roller/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Ground Roller/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Ground Roller/Motor 1 Temperature", motor1Temp.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
  }
}
