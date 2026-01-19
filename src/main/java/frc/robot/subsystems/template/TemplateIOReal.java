package frc.robot.subsystems.template;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TemplateIOReal implements TemplateIO {

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<AngularAcceleration> motorAcceleration;
  private final StatusSignal<Temperature> motorTemp;
  private final StatusSignal<Double> motorClosedLoopGoal;

  public TalonFX motor;

  public TemplateIOReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection statusSignalCollection) {

    motor = new TalonFX(-1, rioCanbus);

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
  }
}
