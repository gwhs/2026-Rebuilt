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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class ShooterIOKitbot implements ShooterIO {

  private final TalonFX motor1; // Right Front

  private final VelocityVoltage velocityRequest1 = new VelocityVoltage(0).withEnableFOC(true);

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<AngularVelocity> motor1Velocity;
  private final StatusSignal<Temperature> motor1Temp;
  private final StatusSignal<Double> motor1ClosedLoopGoal;

  private final Alert motor1NotConnectedAlert =
      new Alert("Shooter Motor 1 Not Connected", AlertType.kError);

  public ShooterIOKitbot(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection statusSignalCollection) {
    motor1 = new TalonFX(ShooterConstants.MOTOR_1_ID, canivoreCanbus);

    motor1Voltage = motor1.getMotorVoltage();
    motor1StatorCurrent = motor1.getStatorCurrent();
    motor1Velocity = motor1.getVelocity();
    motor1Temp = motor1.getDeviceTemp();
    motor1ClosedLoopGoal = motor1.getClosedLoopReference();

    statusSignalCollection.addSignals(
        motor1Voltage, motor1StatorCurrent, motor1Velocity, motor1Temp, motor1ClosedLoopGoal);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, motor1Voltage, motor1StatorCurrent, motor1Velocity, motor1Temp, motor1ClosedLoopGoal);

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 100;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Slot0.kS = 0.2;
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.122;
    talonFXConfig.Slot0.kP = 0.1;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    setUpMotors(talonFXConfig, motor1);
  }

  public void runVoltage(double voltage) {
    motor1.setVoltage(voltage);
  }

  public void enableLeftShooter(boolean enable) {}

  public void enableRightShooter(boolean enable) {}

  public void runVelocity(double rotationsPerSecond) {
    motor1.setControl(velocityRequest1.withVelocity(rotationsPerSecond));
  }

  public void runVelocity(double frontRPS, double backRPS) {
    motor1.setControl(velocityRequest1.withVelocity(frontRPS));
  }

  public double getVelocity() {
    return (motor1Velocity.getValueAsDouble()); // rotations per second
  }

  @SuppressWarnings("resource")
  private boolean setUpMotors(TalonFXConfiguration talonFXConfig, TalonFX motor) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i <= 5; i++) {
      status = motor.getConfigurator().apply(talonFXConfig);
      if (status.isOK()) return true;
    }
    if (!status.isOK()) {
      new Alert(
              "Shooter/Motor "
                  + motor.getDeviceID()
                  + ": Could not configure device. Error:"
                  + status.toString(),
              AlertType.kError)
          .set(true);
    }
    return false;
  }

  public void periodic() {

    DogLog.log("Shooter/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor 1 Velocity", motor1Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor 1 Temperature", motor1Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor 1 Closed Loop Goal", motor1ClosedLoopGoal.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
  }
}
