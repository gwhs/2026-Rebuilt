// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

private IndexerIO indexerIO;

  public IndexerSubsystem (CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection signal) {

    if (RobotBase.isSimulation()) {
      indexerIO = new IndexerIOSim();
    } else {
      indexerIO = new IndexerIOReal(rioCanbus, canivoreCanbus, signal);
    }

  }

  public Command runVoltage(double voltage) {
    return this.runOnce(
        () -> {
          indexerIO.runVoltage(voltage);
        });
  }

  @Override
  public void periodic() {
    indexerIO.periodic();
  }
}
