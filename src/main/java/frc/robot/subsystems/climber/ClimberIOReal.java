// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.indexer.IndexerConstants;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {

    private final TalonFX motor1;
    private final TalonFX motor2;

    private final Alert motor1NotConnectedAlert =
        new Alert ("Climber Motor 1 Not Connected", AlertType.kError);

    private final Alert motor2NotConnectedAlert =
        new Alert ("Climber Motor 2 Not Connected", AlertType.kError);

    public ClimberIOReal(
        CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection statusSignalCollection) {
            
        motor1 = new TalonFX(ClimberConstants.MOTOR_1_ID, rioCanbus);
        motor2 = new TalonFX(ClimberConstants.MOTOR_2_ID, rioCanbus);

        motor1NotConnectedAlert.set(!motor1.isConnected());
        motor2NotConnectedAlert.set(!motor1.isConnected());

        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

        talonFXConfig.CurrentLimits.StatorCurrentLimit = 30;
        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i <= 5; i++) {
        status = motor1.getConfigurator().apply(talonFXConfig);
        if (status.isOK()) break;
    }
        if (!status.isOK()) {
        new Alert("Climber: Could not configure Motor 1. Error" + status.toString(), AlertType.kError)
          .set(true);
    }

        talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i <= 5; i++) {
        status = motor2.getConfigurator().apply(talonFXConfig);
        if (status.isOK()) break;
    }
        if (!status.isOK()) {
        new Alert("Climber: Could not configure Motor 2. Error" + status.toString(), AlertType.kError)
          .set(true);
    }
        }
        
    public void runVoltage(double voltage) {
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }
}
