package frc.robot.subsystems.template;

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

public class TemplateIOReal implements TemplateIO {

  private final TalonFX motor;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<AngularAcceleration> motorAcceleration;
  private final StatusSignal<Temperature> motorTemp;
  private final StatusSignal<Double> motorClosedLoopGoal;

  private final Alert motorNotConnectedAlert = new Alert("Motor 1 Not Connected", AlertType.kError);

  public TemplateIOReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection statusSignalCollection) {

    motor = new TalonFX(TemplateConstants.MOTOR_ID, rioCanbus);

    motorVoltage = motor.getMotorVoltage();
    motorStatorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();
    motorTemp = motor.getDeviceTemp();
    motorAcceleration = motor.getAcceleration();
    motorClosedLoopGoal = motor.getClosedLoopReference();

    statusSignalCollection.addSignals(
        motorVoltage,
        motorStatorCurrent,
        motorVelocity,
        motorTemp,
        motorAcceleration,
        motorClosedLoopGoal);

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfig.CurrentLimits.StatorCurrentLimit = 80;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Slot0.kS = 0.125;
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.1125;
    talonFXConfig.Slot0.kP = 0.5;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i <= 5; i++) {
      status = motor.getConfigurator().apply(talonFXConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      new Alert(
              "Shooter/Motor 1: Could not configure device. Error:" + status.toString(),
              AlertType.kError)
          .set(true);
    }

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }

  @Override
  public void periodic() {
    DogLog.log("the best team ever", 5507);
    motorNotConnectedAlert.set(!motor.isConnected());
  }
}
