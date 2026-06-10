// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.blocker;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlockerSubsystem extends SubsystemBase {

  private BlockerIO blockerIO;

  public static BlockerSubsystem createSim() {
    return new BlockerSubsystem(new BlockerIOSim());
  }

  public static BlockerSubsystem createDisabled() {
    return new BlockerSubsystem(new BlockerIODisabled());
  }

  public static BlockerSubsystem createReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection signal) {
    return new BlockerSubsystem(new BlockerIOReal(rioCanbus, canivoreCanbus, signal));
  }

  /** Creates a new blocker. */
  public BlockerSubsystem(BlockerIO blockerIO) {
    this.blockerIO = blockerIO;
  }

  @Override
  public void periodic() {
    blockerIO.periodic();
    DogLog.log("Blocker/Current Position", blockerIO.getMotor1Position());
  }

  public double getMotor1Position() {
    return blockerIO.getMotor1Position();
  }

  public Command deploy() {
    return this.runOnce(
        () -> {
          blockerIO.runPosition(BlockerConstants.MAX_ROTATION);
        });
  }

  public Command retract() {
    return this.runOnce(
        () -> {
          blockerIO.runPosition(BlockerConstants.MIN_ROTATION);
        });
  }
}
