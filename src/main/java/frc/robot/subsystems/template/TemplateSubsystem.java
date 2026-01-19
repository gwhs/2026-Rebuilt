package frc.robot.subsystems.template;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TemplateSubsystem extends SubsystemBase {
  private TemplateIO template;

  public TemplateSubsystem(CANBus rioCanbus, CANBus canivoreCanbus, StatusSignalCollection signal) {
    if (RobotBase.isSimulation()) {
      template = new TemplateIOSim();
    } else {
      template = new TemplateIOReal(rioCanbus, canivoreCanbus, signal);
    }
  }

  @Override
  public void periodic() {
    template.periodic();
  }
}
