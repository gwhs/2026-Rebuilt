// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.blocker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.climber.ClimberConstants;

public class BlockerIOReal implements BlockerIO {

  private final TalonFX motor1;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);

  private final Alert motor1NotConnectedAlert =
      new Alert("Climber Motor 1 Not Connected", AlertType.kError);

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<AngularAcceleration> motor1Acceleration;
  private final StatusSignal<Temperature> motor1Temp;
  private final StatusSignal<Double> motor1ClosedLoopGoal;
  private final StatusSignal<Angle> motor1Position;

  public BlockerIOReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection statusSignalCollection) {

    motor1 = new TalonFX(ClimberConstants.MOTOR_1_ID, canivoreCanbus);

    motor1Voltage = motor1.getMotorVoltage();
    motor1StatorCurrent = motor1.getStatorCurrent();
    motor1Temp = motor1.getDeviceTemp();
    motor1Acceleration = motor1.getAcceleration();
    motor1ClosedLoopGoal = motor1.getClosedLoopReference();
    motor1Position = motor1.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor1Voltage,
        motor1StatorCurrent,
        motor1Temp,
        motor1Acceleration,
        motor1ClosedLoopGoal,
        motor1Position);

    motor1NotConnectedAlert.set(!motor1.isConnected());

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 30;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Slot0.kS = 0; // TODO
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.1125; // TODO
    talonFXConfig.Slot0.kP = 1.5; // TODO
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = BlockerConstants.MAX_ROTATION;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BlockerConstants.MIN_ROTATION;

    talonFXConfig.MotionMagic.MotionMagicAcceleration = 100;
    talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = 100;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i <= 5; i++) {
      status = motor1.getConfigurator().apply(talonFXConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      new Alert("Blocker: Could not configure Motor 1. Error" + status.toString(), AlertType.kError)
          .set(true);
    }
  }

  public void runPosition(double rotation) {
    motor1.setControl(request.withPosition(rotation));
  }

  public double getMotor1Position() {
    return motor1Position.getValueAsDouble();
  }

  public void periodic() {
    BaseStatusSignal.refreshAll(motor1Voltage,
        motor1StatorCurrent,
        motor1Temp,
        motor1Acceleration,
        motor1ClosedLoopGoal,
        motor1Position);
        
    DogLog.log("Blocker/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Blocker/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Blocker/Motor 1 Temperature", motor1Temp.getValueAsDouble());
    DogLog.log("Blocker/Motor 1 Acceleration", motor1Acceleration.getValueAsDouble());
    DogLog.log("Blocker/Motor 1 Closed Loop Goal", motor1ClosedLoopGoal.getValueAsDouble());
    DogLog.log("Blocker/Motor 1 Position", motor1Position.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
  }
}
