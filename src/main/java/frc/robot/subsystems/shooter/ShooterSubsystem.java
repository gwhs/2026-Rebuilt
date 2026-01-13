package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private ShooterIO shooterIO;
 
  public ShooterSubsystem() {
    if (RobotBase.isSimulation()) {
      shooterIO = new ShooterIOSim();
    } else {
      shooterIO = new ShooterIOReal();
    }
  }

  @Override
  public void periodic() {
 
  }
}
