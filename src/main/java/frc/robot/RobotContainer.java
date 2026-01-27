package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.StatusSignalCollection;
import com.pathplanner.lib.commands.PathfindingCommand;
import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.objectDetection.GamePieceTracker;
import frc.robot.subsystems.objectDetection.ObjectDetectionCam;
import frc.robot.subsystems.objectDetection.ObjectDetectionConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.RotationTarget;
import frc.robot.subsystems.swerve.TunerConstants_Anemone;
import frc.robot.subsystems.swerve.TunerConstants_mk4n;
import java.util.Optional;
import java.util.function.BiConsumer;

public class RobotContainer {
  public enum Robot {
    DEV,
    COMP,
    ANEMONE,
    KITBOT
  }

  private final SwerveSubsystem drivetrain;
  public static final CommandXboxController controller = new CommandXboxController(0);

  private final DriveCommand defualtDriveCommand;

  // TODO: update serial numbers
  @SuppressWarnings("resource")
  public static Robot getRobot() {
    if (RobotController.getSerialNumber().equals("032414F0")) {
      return Robot.ANEMONE;
    } else if (RobotController.getSerialNumber().equals("03223849")) {
      return Robot.DEV;
    } else if (RobotController.getSerialNumber().equals("1234")) {
      return Robot.COMP;
    } else if (RobotController.getSerialNumber().equals("123")) {
      return Robot.KITBOT;
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

  private final StatusSignalCollection signalList = new StatusSignalCollection();
  //

  private final RobotVisualizer robovisual = new RobotVisualizer();
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final ShooterSubsystem shooter =
      new ShooterSubsystem(rioCanbus, canivoreCanbus, signalList);

  public RobotContainer(BiConsumer<Runnable, Double> addPeriodic) {

    this.addPeriodic = addPeriodic;

    addPeriodic.accept(
        () -> {
          CANBusStatus status = canivoreCanbus.getStatus();
          DogLog.log("Canivore/Canivore Bus Utilization", status.BusUtilization);
          DogLog.log("Canivore/Status Code on Canivore", status.Status.toString());
        },
        0.5);

    switch (getRobot()) {
      case COMP:
        drivetrain = TunerConstants_Anemone.createDrivetrain();
        break;
      case ANEMONE:
        drivetrain = TunerConstants_Anemone.createDrivetrain();
        break;
      case KITBOT:
        drivetrain = TunerConstants_Anemone.createDrivetrain();
        break;
      case DEV:
        drivetrain = TunerConstants_mk4n.createDrivetrain();
        break;
      default:
        drivetrain = TunerConstants_Anemone.createDrivetrain();
        break;
    }

    defualtDriveCommand = new DriveCommand(drivetrain, controller);

    objDecCam =
        new ObjectDetectionCam(
            "cam2026_01", ObjectDetectionConstants.robotToCam, () -> drivetrain.getState().Pose);

    configureBindings();
    configureAutonomous();
    configureFuelSim();
    drivetrain.setDefaultCommand(defualtDriveCommand);

    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    addPeriodic.accept(() -> {}, 0.5);

    SmartDashboard.putData(
        "auto rotate",
        drivetrain.setRotationCommand(RotationTarget.TST)); // fix rotate wobble when stop
    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  FuelSim.getInstance().clearFuel();
                })
            .withName("Reset Fuel")
            .ignoringDisable(true));
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
  private void configureBindings() {
    controller.leftBumper().whileTrue(drivetrain.temporarilyDisableRotation());
    controller
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Translation3d initPosition =
                      new Translation3d(
                          drivetrain.getState().Pose.getX(),
                          drivetrain.getState().Pose.getY(),
                          0.635);
                  double dst =
                      EagleUtil.getRobotTargetDistance(
                          drivetrain.getState().Pose, FieldConstants.RED_HUB);
                  double d = EagleUtil.getShooterVelocity(dst);
                  DogLog.log("velocity of fuel", d);
                  DogLog.log("distance to tar", dst);

                  double a = drivetrain.getState().Pose.getRotation().getRadians();
                  Translation3d initVelocity =
                      new Translation3d(
                          d * Math.cos(Math.PI / 3) * Math.cos(a),
                          d * Math.cos(Math.PI / 3) * Math.sin(a),
                          d * Math.sin(Math.PI / 3) * Math.sin(Math.PI / 3));
                  FuelSim.getInstance()
                      .spawnFuel(
                          initPosition,
                          initVelocity); // spawns a fuel with a given position and velocity (both
                  // field centric, represented as vectors by Translation3d)
                }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureAutonomous() {
    SmartDashboard.putData("autonomous", autoChooser);
  }

  private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    // instance.spawnStartingFuel();
    instance.registerRobot(
        0.660, // from left to right
        0.711, // from front to back
        0.127, // from floor to top of bumpers
        () -> drivetrain.getState().Pose, // Supplier<Pose2d> of robot pose
        () ->
            drivetrain.getState()
                .Speeds); // Supplier<ChassisSpeeds> of field-centric chassis speeds

    instance.start();
  }

  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    startTime = HALUtil.getFPGATime();

    if (DriverStation.getAlliance().isPresent()) {
      DogLog.log("Alliance", DriverStation.getAlliance().get());
    }

    if (objDecCam != null) {
      objDecCam.updateDetection();
    }

    DogLog.log(
        "Loop Time/Robot Container/objectDetection Cam",
        (HALUtil.getFPGATime() - startTime) / 1000);

    signalList.refreshAll();

    // 2
    DogLog.log(
        "Loop Time/Robot Container/Robot Visualizer", (HALUtil.getFPGATime() - startTime) / 1000);
    robovisual.update();
    startTime = HALUtil.getFPGATime();

    // Log Triggers
    DogLog.log("Current Robot", getRobot().toString());
    DogLog.log("Match Timer", DriverStation.getMatchTime());

    Pose2d r1 = drivetrain.getState().Pose;
    Pose2d r2 = drivetrain.getPose(0.2);
    Translation2d t = FieldConstants.RED_HUB;
    Pose2d rt = EagleUtil.calcAimpoint(r1, r2, t);

    DogLog.log("aimpoint", rt);
    DogLog.log("estPos", r2);
    // log object
    Optional<Pose2d> obj = GamePieceTracker.getGamePiece();

    if (obj.isPresent()) {
      DogLog.log("Object Detection/Fuel Pose", new Pose2d[] {obj.get()}); // ill forget it tommorow
    } else {
      DogLog.log("Object Detection/Fuel Pose", new Pose2d[0]); // ill forget it tommorow
    }

    FuelSim.getInstance().updateSim();
  }
}
