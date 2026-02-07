package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.StatusSignalCollection;
import com.pathplanner.lib.commands.PathfindingCommand;
import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.HubTracker.Shift;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionSubsystem;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.objectDetection.GamePieceTracker;
import frc.robot.subsystems.objectDetection.ObjectDetectionCam;
import frc.robot.subsystems.objectDetection.ObjectDetectionConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.RotationTarget;
import frc.robot.subsystems.swerve.TunerConstants_Anemone;
import frc.robot.subsystems.swerve.TunerConstants_Mk4i;
import frc.robot.subsystems.swerve.TunerConstants_mk4n;
import java.util.Optional;
import java.util.function.BiConsumer;

public class RobotContainer {
  public enum Robot {
    DEV,
    COMP,
    ANEMONE,
    KITBOT,
    SIM
  }

  private final SwerveSubsystem drivetrain;
  public static final CommandXboxController controller = new CommandXboxController(0);

  private final DriveCommand defualtDriveCommand;

  @SuppressWarnings("resource")
  public static Robot getRobot() {
    final String serialNumber = RobotController.getSerialNumber();
    if (RobotBase.isSimulation()) {
      return Robot.SIM;
    } else if (serialNumber.equals("032414F0")) {
      return Robot.ANEMONE;
    } else if (serialNumber.equals("03223849")) {
      return Robot.DEV;
    } else if (serialNumber.equals("1234")) {
      return Robot.COMP;
    } else if (serialNumber.equals("03282BB2")) {
      return Robot.KITBOT;
    } else {
      new Alert(
              "roborio unrecognized. here is the serial number:" + serialNumber,
              Alert.AlertType.kError)
          .set(true);
      ;
      return Robot.COMP;
    }
  }

  private ObjectDetectionCam objDecCam;

  @SuppressWarnings("unused")
  private final BiConsumer<Runnable, Double> addPeriodic;

  private final CANBus rioCanbus = new CANBus("rio");
  private final CANBus canivoreCanbus = new CANBus("CAN_Network");

  private final StatusSignalCollection signalList = new StatusSignalCollection();

  private final RobotVisualizer robovisual = new RobotVisualizer();
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final ShooterSubsystem shooter;
  private final GroundIntakeRollerSubsystem groundIntakeRoller;
  private final GroundIntakeLinearExtensionSubsystem groundIntakeExtension;
  private ClimberSubsystem climber;
  private final IndexerSubsystem indexer;

  public final Trigger isHubActive =
      new Trigger(
          () -> {
            Shift currentShift = HubTracker.getCurrentShift().orElse(Shift.SHIFT_1);
            double timeRemaining =
                HubTracker.timeRemainingInCurrentShift()
                    .orElse(Time.ofBaseUnits(0, Units.Second))
                    .in(Units.Seconds);
            DogLog.log("Hub Status/Time Remaining in Shift", timeRemaining);
            DogLog.log("Hub Status/Current Shift", currentShift.name());

            double upperThreshold = 3;
            double lowerThreshold = 24;

            if (HubTracker.getAutoWinner().orElse(Alliance.Red) == Alliance.Red) {
              // Red Win
              if (EagleUtil.isRedAlliance()) {
                // We are Red
                if (currentShift == Shift.SHIFT_1 || currentShift == Shift.SHIFT_3) {
                  return timeRemaining >= lowerThreshold
                      || timeRemaining <= upperThreshold
                      || HubTracker.isActive();
                }
              } else {
                // We Lose, as Blue
                if (currentShift == Shift.SHIFT_2 || currentShift == Shift.SHIFT_4) {
                  return timeRemaining >= lowerThreshold
                      || timeRemaining <= upperThreshold
                      || HubTracker.isActive();
                }
              }
            } else {
              // Blue Win
              if (!EagleUtil.isRedAlliance()) {
                // We Win, as Blue
                if (currentShift == Shift.SHIFT_1 || currentShift == Shift.SHIFT_3) {
                  return timeRemaining >= lowerThreshold
                      || timeRemaining <= upperThreshold
                      || HubTracker.isActive();
                }
              } else {
                // We Win, as Red
                if (currentShift == Shift.SHIFT_2 || currentShift == Shift.SHIFT_4) {
                  return timeRemaining >= lowerThreshold
                      || timeRemaining <= upperThreshold
                      || HubTracker.isActive();
                }
              }
            }
            return HubTracker.isActive();
          });

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
        shooter =
            ShooterSubsystem.createReal(
                rioCanbus,
                canivoreCanbus,
                signalList,
                drivetrain.poseSupplier(),
                drivetrain::getVirtualTarget);
        climber = ClimberSubsystem.createReal(rioCanbus, canivoreCanbus, signalList);
        indexer = IndexerSubsystem.createReal(rioCanbus, canivoreCanbus, signalList);
        groundIntakeRoller =
            GroundIntakeRollerSubsystem.createReal(rioCanbus, canivoreCanbus, signalList);
        groundIntakeExtension =
            GroundIntakeLinearExtensionSubsystem.createReal(rioCanbus, canivoreCanbus, signalList);
        break;
      case ANEMONE:
        drivetrain = TunerConstants_Anemone.createDrivetrain();
        shooter =
            ShooterSubsystem.createDisabled(
                drivetrain.poseSupplier(), drivetrain::getVirtualTarget);
        climber = ClimberSubsystem.createDisabled();
        indexer = IndexerSubsystem.createDisabled();
        groundIntakeRoller = GroundIntakeRollerSubsystem.createDisabled();
        groundIntakeExtension = GroundIntakeLinearExtensionSubsystem.createDisabled();
        break;
      case KITBOT:
        drivetrain = TunerConstants_Mk4i.createDrivetrain();
        shooter =
            ShooterSubsystem.createDisabled(
                drivetrain.poseSupplier(), drivetrain::getVirtualTarget);
        climber = ClimberSubsystem.createDisabled();
        indexer = IndexerSubsystem.createDisabled();
        groundIntakeRoller = GroundIntakeRollerSubsystem.createDisabled();
        groundIntakeExtension = GroundIntakeLinearExtensionSubsystem.createDisabled();
        break;
      case DEV:
        drivetrain = TunerConstants_mk4n.createDrivetrain();
        shooter =
            ShooterSubsystem.createDisabled(
                drivetrain.poseSupplier(), drivetrain::getVirtualTarget);
        climber = ClimberSubsystem.createDisabled();
        indexer = IndexerSubsystem.createDisabled();
        groundIntakeRoller = GroundIntakeRollerSubsystem.createDisabled();
        groundIntakeExtension = GroundIntakeLinearExtensionSubsystem.createDisabled();
        break;
      case SIM:
        drivetrain = TunerConstants_Anemone.createDrivetrain();
        shooter =
            ShooterSubsystem.createSim(drivetrain.poseSupplier(), drivetrain::getVirtualTarget);
        climber = ClimberSubsystem.createSim();
        indexer = IndexerSubsystem.createSim();
        groundIntakeRoller = GroundIntakeRollerSubsystem.createSim();
        groundIntakeExtension = GroundIntakeLinearExtensionSubsystem.createSim();
        break;
      default:
        drivetrain = TunerConstants_Anemone.createDrivetrain();
        shooter =
            ShooterSubsystem.createReal(
                rioCanbus,
                canivoreCanbus,
                signalList,
                drivetrain.poseSupplier(),
                drivetrain::getVirtualTarget);
        climber = ClimberSubsystem.createReal(rioCanbus, canivoreCanbus, signalList);
        indexer = IndexerSubsystem.createReal(rioCanbus, canivoreCanbus, signalList);
        groundIntakeRoller =
            GroundIntakeRollerSubsystem.createReal(rioCanbus, canivoreCanbus, signalList);
        groundIntakeExtension =
            GroundIntakeLinearExtensionSubsystem.createReal(rioCanbus, canivoreCanbus, signalList);
        break;
    }

    defualtDriveCommand = new DriveCommand(drivetrain, controller);

    objDecCam =
        new ObjectDetectionCam(
            "cam2026_01", ObjectDetectionConstants.robotToCam, () -> drivetrain.getState().Pose);

    configureBindings();
    configureAutonomous();

    drivetrain.setDefaultCommand(defualtDriveCommand);

    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    DogLog.log("Current Robot", getRobot().toString());

    if (RobotBase.isSimulation()) {
      configureFuelSim();
      SmartDashboard.putData(
          Commands.runOnce(
                  () -> {
                    FuelSim.getInstance().clearFuel();
                  })
              .withName("Reset Fuel")
              .ignoringDisable(true));
    }
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
    RobotModeTriggers.disabled().onTrue(disableHandler());
    controller.leftBumper().whileTrue(drivetrain.temporarilyDisableRotation());
    drivetrain.isOnBump.whileTrue(drivetrain.temporarilyDisableRotation());
    controller.rightTrigger().and(drivetrain.isInAllianceZone).whileTrue(shootHub());
    controller.b().whileTrue(agitateGroundIntake());
    controller
        .rightTrigger()
        .and(
            drivetrain
                .isInNeutralZone
                .or(drivetrain.isInOpponentAllianceZone)
                .and(drivetrain.isOnDepotSide))
        .whileTrue(shootDepot());
    controller
        .rightTrigger()
        .and(
            drivetrain
                .isInNeutralZone
                .or(drivetrain.isInOpponentAllianceZone)
                .and(drivetrain.isOnOutpostSide))
        .whileTrue(shootOutpost());
    controller.rightTrigger().onFalse(drivetrain.setRotationCommand(RotationTarget.NORMAL));

    controller.a().whileTrue(unStuck());

    controller
        .y()
        .whileTrue(drivetrain.setShootingRange(true))
        .onFalse(drivetrain.setShootingRange(false));

    drivetrain.isInAllianceZone.onTrue(drivetrain.setRotationCommand(RotationTarget.HUB));
    drivetrain.isInAllianceZone.onTrue(shooter.cruiseControl());

    controller
        .rightBumper()
        .onTrue(drivetrain.setSlowMode(true))
        .onFalse(drivetrain.setSlowMode(false));

    controller.povDown().whileTrue(deployGroundIntake());
    controller.povDown().onFalse(groundIntakeRoller.stopIntake());

    controller.x().whileTrue(defenseMode());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureAutonomous() {
    SmartDashboard.putData("autonomous", autoChooser);
  }

  private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(
        0.660, // from left to right
        0.711, // from front to back
        0.127, // from floor to top of bumpers
        () -> drivetrain.getState().Pose, // Supplier<Pose2d> of robot pose
        () ->
            ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation()));
    // Supplier<ChassisSpeeds> of field-centric chassis speeds

    // Register an intake to remove fuel from the field as a rectangular bounding box
    instance.registerIntake(
        0.350, 0.700, -0.330, 0.330); // robot-centric coordinates for bounding box

    instance.start();
  }

  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    if (objDecCam != null) {
      objDecCam.updateDetection();
    }

    DogLog.log(
        "Loop Time/Robot Container/objectDetection Cam",
        (HALUtil.getFPGATime() - startTime) / 1000);

    if (RobotBase.isReal()) {
      signalList.refreshAll();
    }

    DogLog.log(
        "Loop Time/Robot Container/Robot Visualizer", (HALUtil.getFPGATime() - startTime) / 1000);
    robovisual.update();
    startTime = HALUtil.getFPGATime();

    DogLog.log("Match Timer", DriverStation.getMatchTime());

    Optional<Pose2d> obj = GamePieceTracker.getGamePiece();

    DogLog.log("Hub Status/Is Active", isHubActive.getAsBoolean());

    if (obj.isPresent()) {
      DogLog.log("Object Detection/Fuel Pose", new Pose2d[] {obj.get()}); // ill forget it tommorow
    } else {
      DogLog.log("Object Detection/Fuel Pose", new Pose2d[0]); // ill forget it tommorow
    }

    double fuelInHopper = FuelSim.getFuelInHopper();

    DogLog.log("number of fuels in hopper", fuelInHopper);
  }

  private Command disableHandler() {
    return Commands.sequence(
            shooter.runVoltage(0.0), drivetrain.setRotationCommand(RotationTarget.NORMAL))
        .ignoringDisable(true);
  }

  public Command shootHub() {
    return Commands.parallel(
        drivetrain.setRotationCommand(RotationTarget.HUB),
        shooter.cruiseControl(),
        drivetrain.setSlowMode(0.5, 0.5)
        Commands.parallel(indexer.index(), EagleUtil.shootInSim(drivetrain))
            .onlyWhile(
                shooter
                    .isAtGoalVelocity_Hub
                    .and(drivetrain.isFacingGoal)
                    .and(isHubActive)
                    .or(controller.leftTrigger()))
            .repeatedly());
  }

  public Command shootDepot() {
    return Commands.parallel(
        drivetrain.setRotationCommand(RotationTarget.PASSING_DEPOT_SIDE),
        shooter.cruiseControl(),
        drivetrain.setSlowMode(0.5, 0.5)
        Commands.parallel(indexer.index(), EagleUtil.shootInSim(drivetrain))
            .onlyWhile(
                shooter
                    .isAtGoalVelocity_Passing
                    .and(drivetrain.isFacingGoalPassing)
                    .or(controller.leftTrigger()))
            .repeatedly());
  }

  public Command shootOutpost() {
    return Commands.parallel(
        drivetrain.setRotationCommand(RotationTarget.PASSING_OUTPOST_SIDE),
        shooter.cruiseControl(),
        drivetrain.setSlowMode(0.5, 0.5)
        Commands.parallel(indexer.index(), EagleUtil.shootInSim(drivetrain))
            .onlyWhile(
                shooter
                    .isAtGoalVelocity_Passing
                    .and(drivetrain.isFacingGoalPassing)
                    .or(controller.leftTrigger()))
            .repeatedly());
  }

  public Command unStuck() {
    return Commands.parallel(
        indexer.reverse(), groundIntakeRoller.reverseIntake(), groundIntakeExtension.extend());
  }

  public Command deployGroundIntake() {
    return Commands.parallel(
        groundIntakeRoller.startIntake(),
        groundIntakeExtension.extend(),
        drivetrain.temporarilyDisableRotation().onlyWhile(controller.rightTrigger().negate()));
  }

  public Command defenseMode() {
    return Commands.parallel(
        drivetrain.swerveX(), groundIntakeExtension.retract(), groundIntakeRoller.stopIntake());
  }

  public Command agitateGroundIntake() {
    return Commands.sequence(
            groundIntakeExtension.extend(), Commands.waitSeconds(.5),
            groundIntakeExtension.retract(), Commands.waitSeconds(.5))
        .repeatedly();
  }
}
