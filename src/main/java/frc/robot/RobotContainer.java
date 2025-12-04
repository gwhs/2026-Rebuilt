package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.pathplanner.lib.commands.PathfindingCommand;
import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.DriveCommand;
import frc.robot.generated.TunerConstants_Comp;
import frc.robot.generated.TunerConstants_WALLE;
import frc.robot.generated.TunerConstants_practiceDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.aprilTagCam.AprilTagCam;
import frc.robot.subsystems.aprilTagCam.AprilTagCamConstants;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class RobotContainer {

  private static Alert roborioError =
      new Alert(
          "roborio unrecognized. here is the serial number:" + RobotController.getSerialNumber(),
          Alert.AlertType.kError);

  public enum Robot {
    WALLE,
    DEV,
    COMP
  }

  public enum RobotState {
    IDLE,
    INTAKE,
    PREPSCORE
  }

  public static RobotState robotState = RobotState.IDLE;

  private final CommandSwerveDrivetrain drivetrain;
  public static final Trigger IS_IDLE = new Trigger(() -> robotState == RobotState.IDLE);
  public static final Trigger IS_INTAKE = new Trigger(() -> robotState == RobotState.INTAKE);
  public static final Trigger IS_PREPSCORE = new Trigger(() -> robotState == RobotState.PREPSCORE);

  public static Robot getRobot() {
    if (RobotController.getSerialNumber().equals("032414F0")) {
      return Robot.COMP;
    } else if (RobotController.getSerialNumber().equals("0323CA18")) {
      return Robot.DEV;
    } else if (RobotController.getSerialNumber().equals("03223849")) {
      return Robot.WALLE;
    } else {
      roborioError.set(true);
      return Robot.COMP;
    }
  }

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final Telemetry logger =
      new Telemetry(TunerConstants_Comp.kSpeedAt12Volts.in(MetersPerSecond));

  public static final Trigger IS_DISABLED = new Trigger(() -> DriverStation.isDisabled());
  public static final Trigger IS_TELEOP = new Trigger(() -> DriverStation.isTeleopEnabled());
  public static final Trigger BATTERY_BROWN_OUT = new Trigger(() -> RobotController.isBrownedOut());

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private AprilTagCam frontLeftCam;

  private AprilTagCam frontRightCam;

  private AprilTagCam backRightCam;

  private AprilTagCam elevatorCam;

  @SuppressWarnings("unused")
  private final BiConsumer<Runnable, Double> addPeriodic;

  public RobotContainer(BiConsumer<Runnable, Double> addPeriodic) {

    this.addPeriodic = addPeriodic;

    switch (getRobot()) {
      case COMP:
        drivetrain = TunerConstants_Comp.createDrivetrain();
        frontLeftCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_LEFT_CAMERA_COMP_NAME,
                AprilTagCamConstants.FRONT_LEFT_CAMERA_LOCATION_COMP,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getPose(),
                () -> drivetrain.getState().Speeds);

        frontRightCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_RIGHT_CAMERA_COMP_NAME,
                AprilTagCamConstants.FRONT_RIGHT_CAMERA_LOCATION_COMP,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getPose(),
                () -> drivetrain.getState().Speeds);

        backRightCam =
            new AprilTagCam(
                AprilTagCamConstants.BACK_RIGHT_CAMERA_COMP_NAME,
                AprilTagCamConstants.BACK_RIGHT_CAMERA_LOCATION_COMP,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getPose(),
                () -> drivetrain.getState().Speeds);

        // elevatorCam =
        //     new AprilTagCam(
        //         AprilTagCamConstants.ELEVATOR_CAMERA_COMP_NAME,
        //         AprilTagCamConstants.ELEVATOR_CAMERA_LOCATION_COMP,
        //         drivetrain::addVisionMeasurent,
        //         () -> drivetrain.getPose(),
        //         () -> drivetrain.getState().Speeds);

        break;
      case DEV:
        drivetrain = TunerConstants_practiceDrivetrain.createDrivetrain();
        frontLeftCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_LEFT_CAMERA_DEV_NAME,
                AprilTagCamConstants.FRONT_LEFT_CAMERA_LOCATION_DEV,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getPose(),
                () -> drivetrain.getState().Speeds);

        frontRightCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_RIGHT_CAMERA_DEV_NAME,
                AprilTagCamConstants.FRONT_RIGHT_CAMERA_LOCATION_DEV,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getPose(),
                () -> drivetrain.getState().Speeds);
        break;
      case WALLE:
        drivetrain = TunerConstants_WALLE.createDrivetrain();
        break;
      default:
        drivetrain = TunerConstants_Comp.createDrivetrain(); // Fallback
        break;
    }

    DriveCommand driveCommand =
        new DriveCommand(
            m_driverController,
            drivetrain,
            () -> {
              return 42.0;
            });

    configureAutonomous();
    configureBindings();

    // Default Commands
    drivetrain.setDefaultCommand(driveCommand);

    drivetrain.registerTelemetry(logger::telemeterize);

    PathfindingCommand.warmupCommand().schedule();

    // Calculate reef setpoints at startup
    EagleUtil.calculateBlueReefSetPoints();
    EagleUtil.calculateRedReefSetPoints();

    addPeriodic.accept(
        () -> {
          CANBusStatus status = TunerConstants_Comp.kCANBus.getStatus();
          DogLog.log("Canivore/Canivore Bus Utilization", status.BusUtilization);
          DogLog.log("Canivore/Status Code on Canivore", status.Status.toString());
        },
        0.5);
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

  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    DogLog.log("nearest", EagleUtil.closestReefSetPoint(drivetrain.getPose(), 0));
    // 1
    DogLog.log(
        "Loop Time/Robot Container/Log Closest Reef Set Point",
        (HALUtil.getFPGATime() - startTime) / 1000);

    startTime = HALUtil.getFPGATime();

    // 2
    DogLog.log(
        "Loop Time/Robot Container/Robot Visualizer", (HALUtil.getFPGATime() - startTime) / 1000);

    startTime = HALUtil.getFPGATime();

    if (frontLeftCam != null) {
      frontLeftCam.updatePoseEstim();
      // 3
      DogLog.log("Loop Time/Robot Container/Cam3", (HALUtil.getFPGATime() - startTime) / 1000);

      startTime = HALUtil.getFPGATime();
    }
    if (frontRightCam != null) {
      frontRightCam.updatePoseEstim();
      // 4
      DogLog.log("Loop Time/Robot Container/Cam4", (HALUtil.getFPGATime() - startTime) / 1000);
    }
    if (backRightCam != null) {
      backRightCam.updatePoseEstim();
    }
    if (elevatorCam != null) {
      elevatorCam.updatePoseEstim();
    }

    startTime = HALUtil.getFPGATime();

    // Log Triggers
    DogLog.log("Trigger/Is Disabled", IS_DISABLED.getAsBoolean());
    DogLog.log("Trigger/Is Telop", IS_TELEOP.getAsBoolean());
    DogLog.log("Current Robot", getRobot().toString());
    DogLog.log("Match Timer", DriverStation.getMatchTime());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureAutonomous() {}

  public Pose2d getRobotPose() {
    return drivetrain.getPose();
  }

  /**
   * this is a wrapper for the command of the same name
   *
   * @param Pose pose to go to
   * @return run the command
   */
  public Command alignToPose(Supplier<Pose2d> Pose) {
    return new AlignToPose(
        Pose,
        drivetrain,
        () -> {
          return 42.0;
        },
        m_driverController);
  }

  public Command unPrepClimbCommand() {
    return Commands.sequence();
  }

  public Command climb() {
    return Commands.sequence();
  }
}
