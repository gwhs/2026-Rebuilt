package frc.robot.subsystems.groundIntakeLinearExtension;

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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class GroundIntakeLinearExtensionIOReal implements GroundIntakeLinearExtensionIO {

  public final TalonFX motor;

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<AngularAcceleration> motorAcceleration;
  private final StatusSignal<Temperature> motorTemp;
  private final StatusSignal<Double> motorClosedLoopGoal;
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<ForwardLimitValue> motorForwardLimit;
  private final StatusSignal<ReverseLimitValue> motorReverseLimit;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);

  private final VoltageOut voltageOutRequest = new VoltageOut(0).withEnableFOC(true);

  private final Alert motorNotConnectedAlert =
      new Alert("Ground Intake Linear Extension Motor 1 Not Connected", AlertType.kError);

  @SuppressWarnings("resource")
  public GroundIntakeLinearExtensionIOReal(
      CANBus rioCanbus, CANBus canivoreCanBus, StatusSignalCollection statusSignalCollection) {
    motor = new TalonFX(GroundIntakeLinearExtensionConstants.MOTOR_ID);

    motorVoltage = motor.getMotorVoltage();
    motorStatorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();
    motorTemp = motor.getDeviceTemp();
    motorAcceleration = motor.getAcceleration();
    motorClosedLoopGoal = motor.getClosedLoopReference();
    motorPosition = motor.getPosition();
    motorForwardLimit = motor.getForwardLimit();
    motorReverseLimit = motor.getReverseLimit();

    statusSignalCollection.addSignals(
        motorVoltage,
        motorStatorCurrent,
        motorVelocity,
        motorTemp,
        motorAcceleration,
        motorClosedLoopGoal,
        motorPosition,
        motorForwardLimit,
        motorReverseLimit);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motorVoltage,
        motorStatorCurrent,
        motorVelocity,
        motorTemp,
        motorAcceleration,
        motorClosedLoopGoal,
        motorPosition,
        motorForwardLimit,
        motorReverseLimit);

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 40;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        GroundIntakeLinearExtensionConstants.MAX_ROTATION;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        GroundIntakeLinearExtensionConstants.MIN_ROTATION;

    talonFXConfig.MotionMagic.MotionMagicAcceleration =
        GroundIntakeLinearExtensionConstants.MAX_ACCELERATION;
    talonFXConfig.MotionMagic.MotionMagicCruiseVelocity =
        GroundIntakeLinearExtensionConstants.MAX_VELOCITY;

    talonFXConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    talonFXConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    talonFXConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;

    talonFXConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    talonFXConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    talonFXConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

    talonFXConfig.Slot0.kP = 0.5;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    talonFXConfig.Slot0.kS = 0.125;
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.1125;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i <= 5; i++) {
      status = motor.getConfigurator().apply(talonFXConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      new Alert(
              "GroundIntakeLinearExtension/Motor 1: Could not configure device. Error:"
                  + status.toString(),
              AlertType.kError)
          .set(true);
    }

    setPosition(0);
  }

  @Override
  public void periodic() {
    DogLog.log("GroundIntakeLinearExtension/Motor 1 Voltage", motorVoltage.getValueAsDouble());
    DogLog.log(
        "GroundIntakeLinearExtension/Motor 1 Stator Current",
        motorStatorCurrent.getValueAsDouble());
    DogLog.log("GroundIntakeLinearExtension/Motor 1 Velocity", motorVelocity.getValueAsDouble());
    DogLog.log("GroundIntakeLinearExtension/Motor 1 Temperature", motorTemp.getValueAsDouble());
    DogLog.log(
        "GroundIntakeLinearExtension/Motor 1 Acceleration", motorAcceleration.getValueAsDouble());
    DogLog.log(
        "GroundIntakeLinearExtension/Motor 1 Closed Loop Goal",
        motorClosedLoopGoal.getValueAsDouble());
    DogLog.log("GroundIntakeLinearExtension/Motor 1 Position", motorPosition.getValueAsDouble());
    DogLog.log(
        "GroundIntakeLinearExtension/Limit Switch Value (Forward)",
        motorForwardLimit.getValueAsDouble());
    DogLog.log(
        "GroundIntakeLinearExtension/Limit Switch Value (Reverse)",
        motorReverseLimit.getValueAsDouble());

    motorNotConnectedAlert.set(!motor.isConnected());
  }

  @Override
  public void runVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public void runVoltage(double volts, boolean ignoreSoftwareLimit) {
    motor.setControl(
        voltageOutRequest.withIgnoreSoftwareLimits(ignoreSoftwareLimit).withOutput(volts));
  }

  public void runPosition(double rotation) {
    motor.setControl(request.withPosition(rotation));
  }

  public double getRotation() {
    return motorPosition.getValueAsDouble();
  }

  public void setPosition(double newValue) {
    motor.setPosition(newValue);
  }

  public double getStatorCurrent() {
    return motorStatorCurrent.getValueAsDouble();
  }

  public boolean getForwardLimit() {
    return motorForwardLimit.getValue() == ForwardLimitValue.ClosedToGround;
  }

  public boolean getReverseLimit() {
    return motorReverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
  }
}
