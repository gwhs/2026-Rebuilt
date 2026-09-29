package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.StatusSignalCollection;
import com.pathplanner.lib.commands.FollowPathCommand;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.aprilTagCam.AprilTagCam;
import frc.robot.subsystems.aprilTagCam.AprilTagCamConstants;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionSubsystem;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants_mk5n;
import java.util.function.BiConsumer;

public class RobotContainer {

  private final CANBus rioCanbus = new CANBus("rio");
  private final CANBus canivoreCanbus = new CANBus("CAN_Network");

  private final StatusSignalCollection signalList = new StatusSignalCollection();

  private final SwerveSubsystem drivetrain = TunerConstants_mk5n.createDrivetrain();
  private final GroundIntakeRollerSubsystem groundIntakeRoller =
      GroundIntakeRollerSubsystem.createReal(rioCanbus, canivoreCanbus, signalList);
  private final GroundIntakeLinearExtensionSubsystem groundIntakeExtension =
      GroundIntakeLinearExtensionSubsystem.createReal(rioCanbus, canivoreCanbus, signalList);
  private final IndexerSubsystem indexer =
      IndexerSubsystem.createReal(rioCanbus, canivoreCanbus, signalList);
  private final ShooterSubsystem shooter =
      ShooterSubsystem.createReal(
          rioCanbus,
          canivoreCanbus,
          signalList,
          drivetrain.poseSupplier(),
          drivetrain::getVirtualTarget);

  public static final CommandXboxController controller = new CommandXboxController(0);

  private final DriveCommand defualtDriveCommand = new DriveCommand(drivetrain, controller);

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(groundIntakeExtension);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public final Trigger isHubActive = new Trigger(EagleUtil::isHubActive);

  private AprilTagCam backRightCam =
      new AprilTagCam(
          AprilTagCamConstants.BACK_RIGHT_CAM,
          AprilTagCamConstants.BACK_RIGHT_CAM_LOCATION,
          drivetrain::addVisionMeasurement,
          () -> drivetrain.getCachedState().Pose,
          () -> drivetrain.getCachedState().Speeds);

  private AprilTagCam backLeftCam =
      new AprilTagCam(
          AprilTagCamConstants.BACK_LEFT_CAM,
          AprilTagCamConstants.BACK_LEFT_CAM_LOCATION,
          drivetrain::addVisionMeasurement,
          () -> drivetrain.getCachedState().Pose,
          () -> drivetrain.getCachedState().Speeds);

  private AprilTagCam frontRightCam =
      new AprilTagCam(
          AprilTagCamConstants.FRONT_LEFT_CAM,
          AprilTagCamConstants.FRONT_LEFT_CAM_LOCATION,
          drivetrain::addVisionMeasurement,
          () -> drivetrain.getCachedState().Pose,
          () -> drivetrain.getCachedState().Speeds);

  private AprilTagCam frontLeftCam =
      new AprilTagCam(
          AprilTagCamConstants.FRONT_RIGHT_CAM,
          AprilTagCamConstants.FRONT_RIGHT_CAM_LOCATION,
          drivetrain::addVisionMeasurement,
          () -> drivetrain.getCachedState().Pose,
          () -> drivetrain.getCachedState().Speeds);

  public RobotContainer(BiConsumer<Runnable, Double> addPeriodic) {
    addPeriodic.accept(
        () -> {
          CANBusStatus status = canivoreCanbus.getStatus();
          DogLog.log("Canivore/Canivore Bus Utilization", status.BusUtilization);
          DogLog.log("Canivore/Status Code on Canivore", status.Status.toString());
        },
        0.5);

    configureBindings();
    configureAutonomous();

    drivetrain.setDefaultCommand(defualtDriveCommand);

    CommandScheduler.getInstance()
        .schedule(
            Commands.parallel(
                    FollowPathCommand.warmupCommand(),
                    drivetrain.setSlowMode(false),
                    shooter.stopShooter(),
                    indexer.runVoltage(0),
                    groundIntakeExtension.retractFull(),
                    groundIntakeRoller.stopIntake())
                .withName("Warm Up Command")
                .ignoringDisable(true));

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
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
    return autoChooser.getSelected();
  }

  private void configureAutonomous() {
    SmartDashboard.putData("autonomous", autoChooser);
  }

  public void periodic() {
    backRightCam.updatePoseEstim();
    backLeftCam.updatePoseEstim();
    frontRightCam.updatePoseEstim();
    frontLeftCam.updatePoseEstim();

    signalList.refreshAll();

    robotVisualizer.periodic();
  }
}
