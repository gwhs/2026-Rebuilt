package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.commands.PathfindingCommand;
import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BiConsumer;
import frc.robot.subsystems.objectDetection.GamePieceTracker;
import frc.robot.subsystems.objectDetection.ObjectDetectionCam;
import frc.robot.subsystems.objectDetection.ObjectDetectionConstants;
import java.util.Optional;

public class RobotContainer {

  public enum Robot {
    WALLE,
    DEV,
    COMP
  }

  // TODO: update serial numbers
  @SuppressWarnings("resource")
  public static Robot getRobot() {
    if (RobotController.getSerialNumber().equals("032414F0")) {
      return Robot.COMP;
    } else if (RobotController.getSerialNumber().equals("0323CA18")) {
      return Robot.DEV;
    } else if (RobotController.getSerialNumber().equals("03223849")) {
      return Robot.WALLE;
    } else {
      new Alert(
              "roborio unrecognized. here is the serial number:"
                  + RobotController.getSerialNumber(),
              Alert.AlertType.kError)
          .set(true);
      ;
      return Robot.COMP;
    }
  }

  @SuppressWarnings("unused")

  private ObjectDetectionCam objDecCam;

  private final BiConsumer<Runnable, Double> addPeriodic;

  private final CANBus rioCanbus = new CANBus("rio");
  private final CANBus canivoreCanbus = new CANBus("CANivore");

  private final RobotVisualizer robovisual = new RobotVisualizer();

  public RobotContainer(BiConsumer<Runnable, Double> addPeriodic) {

    this.addPeriodic = addPeriodic;

    switch (getRobot()) {
      default:
        break;
    }

    objDecCam =
    new ObjectDetectionCam(
        "cam2026_01", ObjectDetectionConstants.robotToCam, () -> drivetrain.getPose());

    configureBindings();

    PathfindingCommand.warmupCommand().schedule();

    addPeriodic.accept(() -> {}, 0.5);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.sequence();
  }

  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    startTime = HALUtil.getFPGATime();

    if (objDecCam != null) {
      objDecCam.updateDetection();
    }
    // 4
    DogLog.log("Loop Time/Robot Container/Cam4", (HALUtil.getFPGATime() - startTime) / 1000);

    // 2
    DogLog.log(
        "Loop Time/Robot Container/Robot Visualizer", (HALUtil.getFPGATime() - startTime) / 1000);
    robovisual.update();
    startTime = HALUtil.getFPGATime();

    // Log Triggers
    DogLog.log("Current Robot", getRobot().toString());
    DogLog.log("Match Timer", DriverStation.getMatchTime());


    // log object
    Optional<Pose2d> obj = GamePieceTracker.getGamePiece();

    if (obj.isPresent()) {
      DogLog.log("Object Detection/Coral Pose", new Pose2d[] {obj.get()}); // ill forget it tommorow
    } else {
      DogLog.log("Object Detection/Coral Pose", new Pose2d[0]); // ill forget it tommorow
    }
  }
}
