package frc.robot.subsystems.groundIntakeLinearExtension;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeLinearExtensionSubsystem extends SubsystemBase {
  private GroundIntakeLinearExtensionIO groundIntakeLinearExtensionIO;

  public static GroundIntakeLinearExtensionSubsystem createSim() {
    return new GroundIntakeLinearExtensionSubsystem(new GroundIntakeLinearExtensionIOSim());
  }

  public static GroundIntakeLinearExtensionSubsystem createDisabled() {
    return new GroundIntakeLinearExtensionSubsystem(new GroundIntakeLinearExtensionIODisabled());
  }

  public static GroundIntakeLinearExtensionSubsystem createReal(
      CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection signal) {
    return new GroundIntakeLinearExtensionSubsystem(
        new GroundIntakeLinearExtensionIOReal(rioCanbus, canivoreCanbus, signal));
  }

  public GroundIntakeLinearExtensionSubsystem(
      GroundIntakeLinearExtensionIO groundIntakeLinearExtensionIO) {
    this.groundIntakeLinearExtensionIO = groundIntakeLinearExtensionIO;

    SmartDashboard.putData("Ground Intake Extension Homing Command", homingCommand());
  }

  @Override
  public void periodic() {
    groundIntakeLinearExtensionIO.periodic();
    DogLog.log(
        "GroundIntakeLinearExtension/Current Rotation", groundIntakeLinearExtensionIO.getRotation());
  }

  public Command extend() {
    return this.runOnce(
        () -> {
          groundIntakeLinearExtensionIO.runPosition(
              GroundIntakeLinearExtensionConstants.EXTENSION_ROTATION);
        });
  }

  public Command retract() {
    return this.runOnce(
        () -> {
          groundIntakeLinearExtensionIO.runPosition(
              GroundIntakeLinearExtensionConstants.RETRACT_ROTATION);
        });
  }

  public Command homingCommand() {
    return Commands.sequence(
        this.runOnce(() -> groundIntakeLinearExtensionIO.runVoltage(-2, true)),
        Commands.waitUntil(() -> groundIntakeLinearExtensionIO.getReverseLimit()),
        this.runOnce(() -> groundIntakeLinearExtensionIO.runVoltage(0)),
        this.runOnce(() -> groundIntakeLinearExtensionIO.setPosition(0)));
  }
}
