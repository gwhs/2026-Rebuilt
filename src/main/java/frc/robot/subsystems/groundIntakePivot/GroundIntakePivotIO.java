package frc.robot.subsystems.groundIntakePivot;

public interface GroundIntakePivotIO {
  public void setAngle(double angle);

  public double getPosition();

  public void update();

  public void setVoltage(double volts);

  public void setEmergencyMode(boolean emergency);

  public double getPositionError();

  public double getPIDGoalDegrees();
}
