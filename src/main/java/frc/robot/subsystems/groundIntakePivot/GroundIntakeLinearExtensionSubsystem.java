package frc.robot.subsystems.groundIntakeLinearExtension;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeLinearExtensionSubsystem extends SubsystemBase {
  public static Object GroundIntakeLinearExtensionIOSim;
  private GroundIntakeLinearExtensionIO groundIntakeLinearExtensionIO;
  private double groundIntakeLinearExtensionGoal;

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
}
