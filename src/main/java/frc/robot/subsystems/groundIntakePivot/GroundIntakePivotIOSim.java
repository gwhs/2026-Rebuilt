package frc.robot.subsystems.groundIntakePivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class GroundIntakePivotIOSim implements GroundIntakePivotIO {

  // Simulation of the ground intake pivot
 
  public void setRotation(double rotation) {
  }

  public double getRotation() {
    return 0.0;
  }

  public double getPIDGoalRotation() {
    return 0.0;
  }
  public void update() {
  }

  private boolean m_emergencyMode;

  public boolean getReverseLimit() {
    return false;
  }

  public boolean getForwardLimit() {
    return false;
  }



  @Override
  public void setVoltage(double voltage) {

  }

  @Override
  public void setPosition(double newValue) {
    }
  

  @Override
  public void setEmergencyMode(boolean emergency) {
  }
}