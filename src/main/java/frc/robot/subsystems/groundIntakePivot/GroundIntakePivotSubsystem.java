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
              null, // Use default ramp rate (1 V/s)
              Units.Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> groundIntakePivotIO.setVoltage(volts.in(Volts)), null, this));

  public GroundIntakePivotSubsystem() {
    if (RobotBase.isSimulation()) {
      groundIntakePivotIO = new GroundIntakePivotIOSim();
    } else {
      groundIntakePivotIO = new GroundIntakePivotIOReal();
    }
  }

 public Trigger AT_GOAL_HEIGHT =
      new Trigger(
          () ->
              MathUtil.isNear(0, groundIntakePivotIO.getRotation() - metersToRotations(groundIntakePivotGoal), 0.9));





  @Override
  public void periodic() {
    double startTime = HALUtil.getFPGATime();
    groundIntakePivotIO.update();

   DogLog.log("GroundIntakePivot/rotation", groundIntakePivotIO.getRotation());
    DogLog.log("GroundIntakePivot/meters", rotationsToMeters(groundIntakePivotIO.getRotation()));
    DogLog.log("GroundIntakePivot/Limit Switch Value (Reverse)", groundIntakePivotIO.getReverseLimit());
    DogLog.log("GroundIntakePivot/Limit Switch Value (Forward)", groundIntakePivotIO.getForwardLimit());

    DogLog.log("GroundIntakePivot/Max Height (meter)", GroundIntakePivotConstants.TOP_METER);

    DogLog.log("Loop Time/GroundIntakePivot", (HALUtil.getFPGATime() - startTime) / 1000);
  }

    public Command setHeight(double meters) {
    double clampedMeters = MathUtil.clamp(meters, 0, GroundIntakePivotConstants.TOP_METER);
    return this.runOnce(
            () -> {
              groundIntakePivotIO.setRotation(metersToRotations(clampedMeters));
              groundIntakePivotGoal = clampedMeters;
            })
        .andThen(
            Commands.waitUntil(
                () ->
                    MathUtil.isNear(
                        clampedMeters, rotationsToMeters(groundIntakePivotIO.getRotation()), 0.1)));
  }

  public Command setHeightSupplier(DoubleSupplier meters) {
    return this.runOnce(
            () -> {
              double clampedMeters =
                  MathUtil.clamp(meters.getAsDouble(), 0, GroundIntakePivotConstants.TOP_METER);
              groundIntakePivotIO.setRotation(metersToRotations(clampedMeters));
              groundIntakePivotGoal = clampedMeters;
            })
        .andThen(
            Commands.waitUntil(
                () -> {
                  double clampedMeters =
                      MathUtil.clamp(meters.getAsDouble(), 0, GroundIntakePivotConstants.TOP_METER);
                  return MathUtil.isNear(
                      clampedMeters, rotationsToMeters(groundIntakePivotIO.getRotation()), 0.1);
                }));
  }

  /**
   * @return run the command
   */
  public Command homingCommand() {
    Command whenNotAtBottom =
        this.runOnce(
                () -> {
                  groundIntakePivotIO.setPosition(metersToRotations(GroundIntakePivotConstants.TOP_METER));
                  ;
                  groundIntakePivotIO.setVoltage(-3);
                })
            .andThen(
                Commands.waitUntil(() -> groundIntakePivotIO.getReverseLimit()),
                Commands.runOnce(
                    () -> {
                      groundIntakePivotIO.setVoltage(0);
                      groundIntakePivotIO.setPosition(0);
                    }));

    Command whenAtBottom =
        Commands.runOnce(
            () -> {
              groundIntakePivotIO.setPosition(0);
            });

    return Commands.either(whenNotAtBottom, whenAtBottom, () -> !groundIntakePivotIO.getReverseLimit());
  }

  /**
   * @param rotations the amount of rotations
   * @return the rotations in equivalent meters
   */
  public static double rotationsToMeters(double rotations) {
    return rotations
        / GroundIntakePivotConstants.GEAR_RATIO
        * (GroundIntakePivotConstants.SPROCKET_DIAMETER * Math.PI)
        * 1;
  }

  /**
   * @param meters the amount of meters
   * @return the meters in equivalent rotations
   */
  public static double metersToRotations(double meters) {
    return meters
        / (GroundIntakePivotConstants.SPROCKET_DIAMETER * Math.PI)
        * GroundIntakePivotConstants.GEAR_RATIO
        / 1;
  }

  /**
   * @return the height in meters
   */
  public double getHeightMeters() {
    return rotationsToMeters(groundIntakePivotIO.getRotation());
  }

  /**
   * @param mode the mode to go to
   */
  public void setNeutralMode(NeutralModeValue mode) {
    groundIntakePivotIO.setNeutralMode(mode);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /**
   * @param meters the amount of meters to add
   * @return run the command
   */
  public Command increaseHeight(double meters) {
    return Commands.runOnce(
        () ->
            groundIntakePivotIO.setRotation(
                metersToRotations(
                    MathUtil.clamp(getHeightMeters() + meters, 0, GroundIntakePivotConstants.TOP_METER))));
  }

  /**
   * @param meters the amount of meters to remove
   * @return run the command
   */
  public Command decreaseHeight(double meters) {
    return Commands.runOnce(
        () ->
            groundIntakePivotIO.setRotation(
                metersToRotations(
                    MathUtil.clamp(getHeightMeters() - meters, 0, GroundIntakePivotConstants.TOP_METER))));
  }

  public Command engageEmergencyMode() {
    return Commands.runOnce(() -> groundIntakePivotIO.setEmergencyMode(true));
  }

  public Command exitEmergencyMode() {
    return Commands.runOnce(() -> groundIntakePivotIO.setEmergencyMode(false));
  }
}