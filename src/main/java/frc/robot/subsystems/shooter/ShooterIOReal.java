package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

public class ShooterIOReal implements ShooterIO {

  private final TalonFX motor1;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<AngularVelocity> motor1Velocity;
  private final StatusSignal<AngularAcceleration> motor1Acceleration;
  private final StatusSignal<Temperature> motor1Temp;
  private final StatusSignal<Double> motor1ClosedLoopGoal;

  private final Alert motor1NotConnectedAlert =
      new Alert("Motor 1 Not Connected", AlertType.kError);

  public ShooterIOReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection statusSignalCollection) {
    motor1 = new TalonFX(ShooterConstants.MOTOR_1_ID, rioCanbus);

    motor1Voltage = motor1.getMotorVoltage();
    motor1StatorCurrent = motor1.getStatorCurrent();
    motor1Velocity = motor1.getVelocity();
    motor1Temp = motor1.getDeviceTemp();
    motor1Acceleration = motor1.getAcceleration();
    motor1ClosedLoopGoal = motor1.getClosedLoopReference();

    statusSignalCollection.addSignals(
        motor1Voltage,
        motor1StatorCurrent,
        motor1Velocity,
        motor1Temp,
        motor1Acceleration,
        motor1ClosedLoopGoal);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        motor1Voltage,
        motor1StatorCurrent,
        motor1Velocity,
        motor1Temp,
        motor1Acceleration,
        motor1ClosedLoopGoal);

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 80;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Slot0.kS = 0.25;
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.1125;
    talonFXConfig.Slot0.kP = 2;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i <= 5; i++) {
      status = motor1.getConfigurator().apply(talonFXConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      new Alert("Shooter: Could not configure device. Error:" + status.toString(), AlertType.kError)
          .set(true);
    }
  }

  public void runVoltage(double voltage) {
    motor1.setVoltage(voltage);
  }

  public void runVelocity(double rotationsPerSecond) {
    motor1.setControl(velocityRequest.withVelocity(rotationsPerSecond));
  }

  public double getVelocity() {
    return motor1Velocity.getValueAsDouble(); // rotations per second
  }

  public void periodic() {
    DogLog.log("Shooter/Motor Voltage: ", motor1Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor Stator Current: ", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor Velocity: ", motor1Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor Temperature: ", motor1Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor Acceleration: ", motor1Acceleration.getValueAsDouble());
    DogLog.log("Shooter/Motor Closed Loop Goal: ", motor1ClosedLoopGoal.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
  }
}
