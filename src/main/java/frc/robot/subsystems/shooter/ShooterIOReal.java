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

  private final TalonFX motor1; // Right Front
  private final TalonFX motor2; // Right Back
  private final TalonFX motor3; // Middle Front
  private final TalonFX motor4; // Middle Back
  private final TalonFX motor5; // Left Front
  private final TalonFX motor6; // Left Back

  private final VelocityVoltage velocityRequest1 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest2 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest3 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest4 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest5 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest6 = new VelocityVoltage(0).withEnableFOC(true);

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

  private final StatusSignal<Voltage> motor3Voltage;
  private final StatusSignal<Current> motor3StatorCurrent;
  private final StatusSignal<AngularVelocity> motor3Velocity;
  private final StatusSignal<AngularAcceleration> motor3Acceleration;
  private final StatusSignal<Temperature> motor3Temp;
  private final StatusSignal<Double> motor3ClosedLoopGoal;

  private final StatusSignal<Voltage> motor4Voltage;
  private final StatusSignal<Current> motor4StatorCurrent;
  private final StatusSignal<AngularVelocity> motor4Velocity;
  private final StatusSignal<AngularAcceleration> motor4Acceleration;
  private final StatusSignal<Temperature> motor4Temp;
  private final StatusSignal<Double> motor4ClosedLoopGoal;

  private final StatusSignal<Voltage> motor5Voltage;
  private final StatusSignal<Current> motor5StatorCurrent;
  private final StatusSignal<AngularVelocity> motor5Velocity;
  private final StatusSignal<AngularAcceleration> motor5Acceleration;
  private final StatusSignal<Temperature> motor5Temp;
  private final StatusSignal<Double> motor5ClosedLoopGoal;

  private final StatusSignal<Voltage> motor6Voltage;
  private final StatusSignal<Current> motor6StatorCurrent;
  private final StatusSignal<AngularVelocity> motor6Velocity;
  private final StatusSignal<AngularAcceleration> motor6Acceleration;
  private final StatusSignal<Temperature> motor6Temp;
  private final StatusSignal<Double> motor6ClosedLoopGoal;

  private final Alert motor1NotConnectedAlert =
      new Alert("Shooter Motor 1 Not Connected", AlertType.kError);
  private final Alert motor2NotConnectedAlert =
      new Alert("Shooter Motor 2 Not Connected", AlertType.kError);
  private final Alert motor3NotConnectedAlert =
      new Alert("Shooter Motor 3 Not Connected", AlertType.kError);
  private final Alert motor4NotConnectedAlert =
      new Alert("Shooter Motor 4 Not Connected", AlertType.kError);
  private final Alert motor5NotConnectedAlert =
      new Alert("Shooter Motor 5 Not Connected", AlertType.kError);
  private final Alert motor6NotConnectedAlert =
      new Alert("Shooter Motor 6 Not Connected", AlertType.kError);

  public ShooterIOReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection statusSignalCollection) {
    motor1 = new TalonFX(ShooterConstants.MOTOR_1_ID, rioCanbus);
    motor2 = new TalonFX(ShooterConstants.MOTOR_2_ID, rioCanbus);
    motor3 = new TalonFX(ShooterConstants.MOTOR_3_ID, rioCanbus);
    motor4 = new TalonFX(ShooterConstants.MOTOR_4_ID, rioCanbus);
    motor5 = new TalonFX(ShooterConstants.MOTOR_5_ID, rioCanbus);
    motor6 = new TalonFX(ShooterConstants.MOTOR_6_ID, rioCanbus);

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

    motor3Voltage = motor3.getMotorVoltage();
    motor3StatorCurrent = motor3.getStatorCurrent();
    motor3Velocity = motor3.getVelocity();
    motor3Temp = motor3.getDeviceTemp();
    motor3Acceleration = motor3.getAcceleration();
    motor3ClosedLoopGoal = motor3.getClosedLoopReference();

    motor4Voltage = motor4.getMotorVoltage();
    motor4StatorCurrent = motor4.getStatorCurrent();
    motor4Velocity = motor4.getVelocity();
    motor4Temp = motor4.getDeviceTemp();
    motor4Acceleration = motor4.getAcceleration();
    motor4ClosedLoopGoal = motor4.getClosedLoopReference();

    motor5Voltage = motor5.getMotorVoltage();
    motor5StatorCurrent = motor5.getStatorCurrent();
    motor5Velocity = motor5.getVelocity();
    motor5Temp = motor5.getDeviceTemp();
    motor5Acceleration = motor5.getAcceleration();
    motor5ClosedLoopGoal = motor5.getClosedLoopReference();

    motor6Voltage = motor6.getMotorVoltage();
    motor6StatorCurrent = motor6.getStatorCurrent();
    motor6Velocity = motor6.getVelocity();
    motor6Temp = motor6.getDeviceTemp();
    motor6Acceleration = motor6.getAcceleration();
    motor6ClosedLoopGoal = motor6.getClosedLoopReference();

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
        motor2ClosedLoopGoal,
        motor3Voltage,
        motor3StatorCurrent,
        motor3Velocity,
        motor3Temp,
        motor3Acceleration,
        motor3ClosedLoopGoal,
        motor4Voltage,
        motor4StatorCurrent,
        motor4Velocity,
        motor4Temp,
        motor4Acceleration,
        motor4ClosedLoopGoal,
        motor5Voltage,
        motor5StatorCurrent,
        motor5Velocity,
        motor5Temp,
        motor5Acceleration,
        motor5ClosedLoopGoal,
        motor6Voltage,
        motor6StatorCurrent,
        motor6Velocity,
        motor6Temp,
        motor6Acceleration,
        motor6ClosedLoopGoal);

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
        motor2ClosedLoopGoal,
        motor3Voltage,
        motor3StatorCurrent,
        motor3Velocity,
        motor3Temp,
        motor3Acceleration,
        motor3ClosedLoopGoal,
        motor4Voltage,
        motor4StatorCurrent,
        motor4Velocity,
        motor4Temp,
        motor4Acceleration,
        motor4ClosedLoopGoal,
        motor5Voltage,
        motor5StatorCurrent,
        motor5Velocity,
        motor5Temp,
        motor5Acceleration,
        motor5ClosedLoopGoal,
        motor6Voltage,
        motor6StatorCurrent,
        motor6Velocity,
        motor6Temp,
        motor6Acceleration,
        motor6ClosedLoopGoal);

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

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

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    setUpMotors(talonFXConfig, motor1);

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    setUpMotors(talonFXConfig, motor2);

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    setUpMotors(talonFXConfig, motor3);

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    setUpMotors(talonFXConfig, motor4);

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    setUpMotors(talonFXConfig, motor5);

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    setUpMotors(talonFXConfig, motor6);
  }

  public void runVoltage(double voltage) {
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
    motor3.setVoltage(voltage);
    motor4.setVoltage(voltage);
    motor5.setVoltage(voltage);
    motor6.setVoltage(voltage);
  }

  public void runVelocity(double rotationsPerSecond) {
    motor1.setControl(velocityRequest1.withVelocity(rotationsPerSecond));
    motor2.setControl(velocityRequest2.withVelocity(rotationsPerSecond));
    motor3.setControl(velocityRequest3.withVelocity(rotationsPerSecond));
    motor4.setControl(velocityRequest4.withVelocity(rotationsPerSecond));
    motor5.setControl(velocityRequest5.withVelocity(rotationsPerSecond));
    motor6.setControl(velocityRequest6.withVelocity(rotationsPerSecond));
  }

  public void runVelocity(double frontRPS, double backRPS) {
    motor1.setControl(velocityRequest1.withVelocity(frontRPS));
    motor2.setControl(velocityRequest2.withVelocity(backRPS));
    motor3.setControl(velocityRequest3.withVelocity(frontRPS));
    motor4.setControl(velocityRequest4.withVelocity(backRPS));
    motor5.setControl(velocityRequest5.withVelocity(frontRPS));
    motor6.setControl(velocityRequest6.withVelocity(backRPS));
  }

  public double getVelocity() {
    return (motor1Velocity.getValueAsDouble()
            + motor2Velocity.getValueAsDouble()
            + motor3Velocity.getValueAsDouble()
            + motor4Velocity.getValueAsDouble()
            + motor5Velocity.getValueAsDouble()
            + motor6Velocity.getValueAsDouble())
        / 6; // rotations per second
  }

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
    DogLog.log("Shooter/Motor 1 Acceleration", motor1Acceleration.getValueAsDouble());
    DogLog.log("Shooter/Motor 1 Closed Loop Goal", motor1ClosedLoopGoal.getValueAsDouble());

    DogLog.log("Shooter/Motor 2 Voltage", motor2Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor 2 Stator Current", motor2StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor 2 Velocity", motor2Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor 2 Temperature", motor2Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor 2 Acceleration", motor2Acceleration.getValueAsDouble());
    DogLog.log("Shooter/Motor 2 Closed Loop Goal", motor2ClosedLoopGoal.getValueAsDouble());

    DogLog.log("Shooter/Motor 3 Voltage", motor3Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor 3 Stator Current", motor3StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor 3 Velocity", motor3Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor 3 Temperature", motor3Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor 3 Acceleration", motor3Acceleration.getValueAsDouble());
    DogLog.log("Shooter/Motor 3 Closed Loop Goal", motor3ClosedLoopGoal.getValueAsDouble());

    DogLog.log("Shooter/Motor 4 Voltage", motor4Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor 4 Stator Current", motor4StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor 4 Velocity", motor4Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor 4 Temperature", motor4Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor 4 Acceleration", motor4Acceleration.getValueAsDouble());
    DogLog.log("Shooter/Motor 4 Closed Loop Goal", motor4ClosedLoopGoal.getValueAsDouble());

    DogLog.log("Shooter/Motor 5 Voltage", motor5Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor 5 Stator Current", motor5StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor 5 Velocity", motor5Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor 5 Temperature", motor5Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor 5 Acceleration", motor5Acceleration.getValueAsDouble());
    DogLog.log("Shooter/Motor 5 Closed Loop Goal", motor5ClosedLoopGoal.getValueAsDouble());

    DogLog.log("Shooter/Motor 6 Voltage", motor6Voltage.getValueAsDouble());
    DogLog.log("Shooter/Motor 6 Stator Current", motor6StatorCurrent.getValueAsDouble());
    DogLog.log("Shooter/Motor 6 Velocity", motor6Velocity.getValueAsDouble());
    DogLog.log("Shooter/Motor 6 Temperature", motor6Temp.getValueAsDouble());
    DogLog.log("Shooter/Motor 6 Acceleration", motor6Acceleration.getValueAsDouble());
    DogLog.log("Shooter/Motor 6 Closed Loop Goal", motor6ClosedLoopGoal.getValueAsDouble());

    DogLog.log(
        "Shooter/Motor 1 and 2 Velocity Difference",
        motor1Velocity.getValueAsDouble() - motor2Velocity.getValueAsDouble());
    DogLog.log(
        "Shooter/Motor 3 and 4 Velocity Difference",
        motor3Velocity.getValueAsDouble() - motor4Velocity.getValueAsDouble());
    DogLog.log(
        "Shooter/Motor 5 and 6 Velocity Difference",
        motor5Velocity.getValueAsDouble() - motor6Velocity.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor1.isConnected());
    motor2NotConnectedAlert.set(!motor2.isConnected());
    motor3NotConnectedAlert.set(!motor3.isConnected());
    motor4NotConnectedAlert.set(!motor4.isConnected());
    motor5NotConnectedAlert.set(!motor5.isConnected());
    motor6NotConnectedAlert.set(!motor6.isConnected());
  }
}
