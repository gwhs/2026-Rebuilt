package frc.robot.subsystems.template;

import edu.wpi.first.wpilibj.RobotBase;

public class TemplateSubsystem {
	private TemplateIO template;

	public TemplateSubsystem() {
		if (RobotBase.isSimulation()) {
			template = new TemplateIOSim();
		}
		else {
			template = new TemplateIOReal();
		}
	}

}
