package frc.robot.subsystems.groundIntakePivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedGroundIntakePivotSim;

public class GroundIntakePivotIOSim implements GroundIntakePivotIO {
  private SingleJointedGroundIntakePivotSim groundIntakePivotSim =
      new SingleJointedGroundIntakePivotSim(
          DCMotor.getFalcon500Foc(1),
          GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_GEAR_RATIO,
          0.1,
          1,
          Units.degreesToRadians(GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_LOWER_BOUND),
          Units.degreesToRadians(GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_UPPER_BOUND),
          false,
          Units.degreesToRadians(90));

  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          GroundIntakePivotConstants.MAX_VELOCITY * 360, GroundIntakePivotConstants.MAX_ACCELERATION * 360);
  private ProfiledPIDController pidController = new ProfiledPIDController(.1, 0, 0, constraints);

  private boolean m_emergencyMode;

  public GroundIntakePivotIOSim() {
    pidController.setGoal(90);
  }

  public double getPIDGoalDegrees() {
    return this.pidController.getGoal().position;
  }

  public double getPosition() {
    return Units.radiansToDegrees(GroundIntakePivotSim.getAngleRads());
  }

  @Override
  public void setAngle(double angle) {
    if (m_emergencyMode == false) {
      pidController.setGoal(angle);
    }
  }

  public double getPositionError() {
    return pidController.getPositionError();
  }

  /**
   * @param volts how many volts to set to
   */
  public void setVoltage(double volts) {
    if (m_emergencyMode == true) {
      groundIntakePivotSim.setInputVoltage(0);
    } else {
      groundIntakePivotSim.setInputVoltage(volts);
    }
  }

  public void update() {
    groundIntakePivotSim.update(0.20);
    if (m_emergencyMode == false) {
      double pidOutput = pidController.calculate(getPosition());

      groundIntakePivotSim.setInputVoltage(pidOutput);
    }
  }

  @Override
  public void setEmergencyMode(boolean emergency) {
    m_emergencyMode = emergency;
    setVoltage(0);
  }
}
