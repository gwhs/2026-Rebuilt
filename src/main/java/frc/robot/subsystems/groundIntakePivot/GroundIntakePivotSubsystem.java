package frc.robot.subsystems.groundIntakePivot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

public class GroundIntakePivotSubsystem extends SubsystemBase {
  private GroundIntakePivotIO groundIntakePivotIO;
  private double groundIntakePivotGoal;

  // Make AT_GOAL_ANGLE lazy to avoid NPE
  public Trigger AT_GOAL_ANGLE() {
    return new Trigger(
        () ->
            groundIntakePivotIO != null
                && MathUtil.isNear(groundIntakePivotGoal, groundIntakePivotIO.getPosition(), 3.0));
  }

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // default ramp rate
              Units.Volts.of(4), // Reduce dynamic step voltage
              null, // default timeout
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> groundIntakePivotIO.setVoltage(volts.in(Volts)), null, this));

  public GroundIntakePivotSubsystem() {
    if (RobotBase.isSimulation()) {
      groundIntakePivotIO = new GroundIntakePivotIOSim();
    } else {
      groundIntakePivotIO = new GroundIntakePivotIOReal();
    }
  }

  /** Drives the groundIntakePivot to a given angle (degrees) */
  public Command setAngle(double angle) {
    double clampedAngle =
        MathUtil.clamp(
            angle,
            GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_LOWER_BOUND,
            GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_UPPER_BOUND);

    return this.runOnce(
            () -> {
              groundIntakePivotIO.setAngle(clampedAngle);
              groundIntakePivotGoal = clampedAngle;
            })
        .andThen(
            Commands.waitUntil(
                () -> MathUtil.isNear(clampedAngle, groundIntakePivotIO.getPosition(), 1.0)));
  }

  /** Drives the groundIntakePivot based on a supplier (degrees) */
  public Command setAngleSupplier(DoubleSupplier angleSupplier) {
    return this.runOnce(
            () -> {
              double clampedAngle =
                  MathUtil.clamp(
                      angleSupplier.getAsDouble(),
                      GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_LOWER_BOUND,
                      GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_UPPER_BOUND);
              groundIntakePivotIO.setAngle(clampedAngle); // FIXED typo here
              groundIntakePivotGoal = clampedAngle;
            })
        .andThen(
            Commands.waitUntil(
                () -> {
                  double clampedAngle =
                      MathUtil.clamp(
                          angleSupplier.getAsDouble(),
                          GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_LOWER_BOUND,
                          GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_UPPER_BOUND);
                  return MathUtil.isNear(clampedAngle, groundIntakePivotIO.getPosition(), 1.0);
                }));
  }

  @Override
  public void periodic() {
    double startTime = HALUtil.getFPGATime();
    groundIntakePivotIO.update();

    DogLog.log("GroundIntakePivot/angle", groundIntakePivotIO.getPosition());
    DogLog.log("LoopTime/GroundIntakePivot", (HALUtil.getFPGATime() - startTime) / 1000.0);
  }

  /**
   * @return the current angle (degrees)
   */
  public double getAngle() {
    return groundIntakePivotIO.getPosition();
  }
}
