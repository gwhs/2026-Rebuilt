package frc.robot.subsystems.groundIntakeLinearExtension;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeLinearExtensionSubsystem extends SubsystemBase {
  private GroundIntakeLinearExtensionIO groundIntakeLinearExtensionIO;

  public GroundIntakeLinearExtensionSubsystem(
      CANBus rioCanbus, CANBus canivoreCanBus, StatusSignalCollection statusSignalCollection) {
    if (RobotBase.isSimulation()) {
      groundIntakeLinearExtensionIO = new GroundIntakeLinearExtensionIOSim();
    } else {
      groundIntakeLinearExtensionIO =
          new GroundIntakeLinearExtensionIOReal(
              canivoreCanBus, canivoreCanBus, statusSignalCollection);
    }
  }

  @Override
  public void periodic() {
    groundIntakeLinearExtensionIO.periodic();
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
