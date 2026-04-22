// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX motor;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);

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

  @SuppressWarnings("resource")
  public ClimberSubsystem(CANBus canBus, StatusSignalCollection statusSignalCollection) {
    motor = new TalonFX(ClimberConstants.MOTOR_1_ID, canBus);

    motor1Voltage = motor.getMotorVoltage();
    motor1StatorCurrent = motor.getStatorCurrent();
    motor1Temp = motor.getDeviceTemp();
    motor1Acceleration = motor.getAcceleration();
    motor1ClosedLoopGoal = motor.getClosedLoopReference();
    motor1Position = motor.getPosition();
    forwardLimit = motor.getForwardLimit();
    reverseLimit = motor.getReverseLimit();

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

    motor1NotConnectedAlert.set(!motor.isConnected());

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
      status = motor.getConfigurator().apply(talonFXConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      new Alert("Climber: Could not configure Motor 1. Error" + status.toString(), AlertType.kError)
          .set(true);
    }
  }

  public Command runVoltage(double voltage) {
    return this.runOnce(
        () -> {
          motor.setVoltage(voltage);
        });
  }

  public Command runPosition(double rotation) {
    return this.runOnce(
            () -> {
              motor.setControl(request.withPosition(rotation));
            })
        .andThen(
            Commands.waitUntil(
                () ->
                    MathUtil.isNear(
                        motor1ClosedLoopGoal.getValueAsDouble(),
                        motor1Position.getValueAsDouble(),
                        0.1)));
  }

  public Command homingCommand() {
    return Commands.none();
    // return Commands.sequence(
    //         Commands.runOnce(() -> climberIO.runVoltage(-3, true)),
    //         Commands.waitUntil(() -> climberIO.getReverseLimitSwitch()),
    //         Commands.runOnce(() -> climberIO.runVoltage(0)),
    //         Commands.runOnce(() -> climberIO.setPosition(0)))
    //     .onlyIf(() -> RobotBase.isReal());
  }

  @Override
  public void periodic() {
    DogLog.log("Climber/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Temperature", motor1Temp.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Acceleration", motor1Acceleration.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Closed Loop Goal", motor1ClosedLoopGoal.getValueAsDouble());
    DogLog.log("Climber/Motor 1 Position", motor1Position.getValueAsDouble());

    motor1NotConnectedAlert.set(!motor.isConnected());
  }

  public double getMotor1Position() {
    return motor1Position.getValueAsDouble();
  }

  /*
   * Simulation
   */

  private static final double kGearRatio = 100.0;
  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio),
          DCMotor.getKrakenX60Foc(1));

  public void simulationInit() {
    var talonFXSim = motor.getSimState();
    talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  public void simulationPeriodic() {
    var talonFXSim = motor.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
    talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
  }
}
