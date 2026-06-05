// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {

  private final TalonFX motor1;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);

  private final VoltageOut voltageOut = new VoltageOut(0);

  private final Alert motor1NotConnectedAlert =
      new Alert("Climber Motor 1 Not Connected", AlertType.kError);

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<AngularAcceleration> motor1Acceleration;
  private final StatusSignal<Temperature> motor1Temp;
  private final StatusSignal<Double> motor1ClosedLoopGoal;
  private final StatusSignal<Angle> motor1Position;

  private final StatusSignal<ForwardLimitValue> forwardLimit;
  private final StatusSignal<ReverseLimitValue> reverseLimit;

  public ClimberIOReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection statusSignalCollection) {

    motor1 = new TalonFX(ClimberConstants.MOTOR_1_ID, rioCanbus);

    motor1Voltage = motor1.getMotorVoltage();
    motor1StatorCurrent = motor1.getStatorCurrent();
    motor1Temp = motor1.getDeviceTemp();
    motor1Acceleration = motor1.getAcceleration();
    motor1ClosedLoopGoal = motor1.getClosedLoopReference();
    motor1Position = motor1.getPosition();
    forwardLimit = motor1.getForwardLimit();
    reverseLimit = motor1.getReverseLimit();

    statusSignalCollection.addSignals(
        motor1Voltage,
        motor1StatorCurrent,
        motor1Temp,
        motor1Acceleration,
        motor1ClosedLoopGoal,
        motor1Position);

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

    talonFXConfig.Slot0.kS = 0.125;
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.065;
    talonFXConfig.Slot0.kP = 0.5;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.MAX_ROTATION;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.MIN_ROTATION;

    talonFXConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    talonFXConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    talonFXConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;

    talonFXConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    talonFXConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    talonFXConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i <= 5; i++) {
      status = motor1.getConfigurator().apply(talonFXConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      new Alert("Climber: Could not configure Motor 1. Error" + status.toString(), AlertType.kError)
          .set(true);
    }
  }

  public void runVoltage(double voltage) {
    motor1.setVoltage(voltage);
  }

  public void runVoltage(double voltage, boolean ignoreSoftwareLimit) {
    motor1.setControl(voltageOut.withIgnoreSoftwareLimits(ignoreSoftwareLimit).withOutput(voltage));
  }

  public void setPosition(double rotation) {
    motor1.setPosition(rotation);
  }

  public void runPosition(double rotation) {
    motor1.setControl(request.withPosition(rotation));
  }

  public double getMotor1Position() {
    return motor1Position.getValueAsDouble();
  }

  public boolean getReverseLimitSwitch() {
    return reverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
  }

  public void periodic() {
    DogLog.log("Climber/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Temperature", motor1Temp.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Acceleration", motor1Acceleration.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Closed Loop Goal", motor1ClosedLoopGoal.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Position", motor1Position.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
  }
}
