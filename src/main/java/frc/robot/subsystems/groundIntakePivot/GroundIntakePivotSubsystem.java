package frc.robot.subsystems.GroundIntakePivotSubsystem;

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

  public Trigger AT_GOAL_ANGLE =
      new Trigger(() -> MathUtil.isNear(0, groundIntakePivotIO.getPosition() - groundIntakePivotGoal, 3));

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Units.Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism((volts) -> groundIntakePivotIO.setVoltage(volts.in(Volts)), null, this));

  public GroundIntakePivotSubsystem() {
    if (RobotBase.isSimulation()) {
      groundIntakePivotIO = new GroundIntakePivotIOSim();
    } else {
      groundIntakePivotIO = new GroundIntakePivotIOReal();
    }
  }

  /**
   * drives the groundIntakePivot until it reaches the given provided angle
   *
   * @param angle Angle to drive the groundIntakePivot to in degrees
   */
  public Command setAngle(double angle) {
    double clampedAngle =
        MathUtil.clamp(angle, GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_LOWER_BOUND, GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_UPPER_BOUND);
    return this.runOnce(
            () -> {
              groundIntakePivotIO.setAngle(clampedAngle);
              groundIntakePivotGoal = clampedAngle;
            })
        .andThen(Commands.waitUntil(() -> MathUtil.isNear(clampedAngle, groundIntakePivotIO.getPosition(), 1)));
  }

  public Command setAngleSupplier(DoubleSupplier angle) {
    return this.runOnce(
            () -> {
              double clampedAngle =
                  MathUtil.clamp(
                      angle.getAsDouble(),
                      GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_LOWER_BOUND,
                      GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_UPPER_BOUND);
              GroundIntakePivotIO.setAngle(clampedAngle);
              GroundIntakePivotGoal = clampedAngle;
            })
        .andThen(
            Commands.waitUntil(
                () -> {
                  double clampedAngle =
                      MathUtil.clamp(
                          angle.getAsDouble(),
                          GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_LOWER_BOUND,
                          GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_UPPER_BOUND);
                  return MathUtil.isNear(clampedAngle, groundIntakePivotIO.getPosition(), 1);
                }));
  }

  @Override
  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    groundIntakePivotIO.update();
    DogLog.log("GroundIntakePivot/groundIntakePivot angle", groundIntakePivotIO.getPosition());
    DogLog.log("Loop Time/Ground Intake Pivot", (HALUtil.getFPGATime() - startTime) / 1000);
  }

  /**
   * @return the groundIntakePivot's angle
   */
  public double getAngle() {
    return groundIntakePivotIO.getPosition();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /**
   * @param degrees the degrees to add
   * @return run the command
   */
  public Command increaseAngle(double degrees) {
    return Commands.runOnce(
            () -> {
              groundIntakePivotIO.setAngle(groundIntakePivotIO.getPosition() + degrees);
            })
        .andThen(Commands.waitUntil(() -> MathUtil.isNear(groundIntakePivotIO.getPositionError(), 0, 1)));
  }

  /**
   * @param degrees the degrees to decrease to
   * @return run the command
   */
  public Command decreaseAngle(double degrees) {
    return increaseAngle(-degrees);
  }

  public Command engageEmergencyMode() {
    return Commands.runOnce(() -> groundIntakePivotIO.setEmergencyMode(true));
  }

  public Command exitEmergencyMode() {
    return Commands.runOnce(() -> groundIntakePivotIO.setEmergencyMode(false));
  }
}

