package frc.robot.subsystems.groundIntakePivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class GroundIntakePivotIOSim implements GroundIntakePivotIO {

  // Simulation of the ground intake pivot
  private final SingleJointedArmSim groundIntakePivotSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_GEAR_RATIO,
          0.1, // moment of inertia
          1,   // damping
          Units.degreesToRadians(GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_LOWER_BOUND),
          Units.degreesToRadians(GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_UPPER_BOUND),
          false,
          Units.degreesToRadians(90) // initial position
      );

  // Convert MAX_VELOCITY and MAX_ACCELERATION from rotations/sec to radians/sec
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          Units.rotationsToRadians(GroundIntakePivotConstants.MAX_VELOCITY),
          Units.rotationsToRadians(GroundIntakePivotConstants.MAX_ACCELERATION)
      );

  private final ProfiledPIDController pidController = new ProfiledPIDController(0.5, 0, 0, constraints);

  private boolean m_emergencyMode = false;

  public GroundIntakePivotIOSim() {
    // Initialize PID goal to current position
    pidController.setGoal(groundIntakePivotSim.getAngleRads());
  }

  @Override
  public double getPIDGoalDegrees() {
    return Units.radiansToDegrees(pidController.getGoal().position);
  }

  @Override
  public double getPosition() {
    return Units.radiansToDegrees(groundIntakePivotSim.getAngleRads());
  }

  @Override
  public void setAngle(double angleDegrees) {
    if (!m_emergencyMode) {
      pidController.setGoal(Units.degreesToRadians(angleDegrees));
    }
  }

  @Override
  public double getPositionError() {
    // PID position error in degrees
    return Units.radiansToDegrees(pidController.getPositionError());
  }

  @Override
  public void setVoltage(double volts) {
    if (m_emergencyMode) {
      groundIntakePivotSim.setInputVoltage(0);
    } else {
      groundIntakePivotSim.setInputVoltage(volts);
    }
  }

  @Override
  public void update() {
    if (!m_emergencyMode) {
      // PID output in volts (assuming 1:1 scaling)
      double pidOutputVolts = pidController.calculate(groundIntakePivotSim.getAngleRads());
      groundIntakePivotSim.setInputVoltage(pidOutputVolts);
    }

    // Update simulation with 20ms timestep
    groundIntakePivotSim.update(0.020);
  }

  @Override
  public void setEmergencyMode(boolean emergency) {
    m_emergencyMode = emergency;
    setVoltage(0);
  }
}
