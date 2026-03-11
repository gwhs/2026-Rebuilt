package frc.robot.subsystems.groundIntakeLinearExtension;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class GroundIntakePivotIOReal implements GroundIntakeLinearExtensionIO {

  private final TalonFX motor;

  private final CANcoder groundIntakePivotEncoder;

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<AngularAcceleration> motorAcceleration;
  private final StatusSignal<Temperature> motorTemp;
  private final StatusSignal<Double> motorClosedLoopGoal;
  private final StatusSignal<Angle> motorPosition;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);

  private final VoltageOut voltageOutRequest = new VoltageOut(0).withEnableFOC(true);

  private final Alert motorNotConnectedAlert =
      new Alert("Ground Intake Linear Extension Motor 1 Not Connected", AlertType.kError);

  @SuppressWarnings("resource")
  public GroundIntakePivotIOReal(
      CANBus rioCanbus, CANBus canivoreCanBus, StatusSignalCollection statusSignalCollection) {
    motor = new TalonFX(GroundIntakeLinearExtensionConstants.MOTOR_ID, canivoreCanBus);

    groundIntakePivotEncoder =
        new CANcoder(GroundIntakeLinearExtensionConstants.PIVOT_ENCODER_ID, canivoreCanBus);

    motorVoltage = motor.getMotorVoltage();
    motorStatorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();
    motorTemp = motor.getDeviceTemp();
    motorAcceleration = motor.getAcceleration();
    motorClosedLoopGoal = motor.getClosedLoopReference();
    motorPosition = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motorVoltage,
        motorStatorCurrent,
        motorVelocity,
        motorTemp,
        motorAcceleration,
        motorClosedLoopGoal,
        motorPosition);

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 40;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Feedback.FeedbackRotorOffset = 0;
    talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    talonFXConfig.Feedback.FeedbackRemoteSensorID =
        GroundIntakeLinearExtensionConstants.PIVOT_ENCODER_ID;
    talonFXConfig.Feedback.SensorToMechanismRatio = 36.0 / 18.0;
    talonFXConfig.Feedback.RotorToSensorRatio = 42.0 / 12.0 * 42.0 / 38.0 * 62.0 / 18.0;

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

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    encoderConfig.MagnetSensor.MagnetOffset = -0.124512 + 0.0627;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    for (int i = 0; i < 5; i++) {
      status = groundIntakePivotEncoder.getConfigurator().apply(encoderConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not Configure this device: " + status.toString());
    }
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

    motorNotConnectedAlert.set(!motor.isConnected());

    BaseStatusSignal.refreshAll(
        motorVoltage,
        motorStatorCurrent,
        motorVelocity,
        motorTemp,
        motorAcceleration,
        motorClosedLoopGoal,
        motorPosition);
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
    return true;
  }

  public boolean getReverseLimit() {
    return true;
  }
}
