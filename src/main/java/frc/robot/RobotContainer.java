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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.HubTracker.Shift;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autonomous.DepotPathAuto_1c;
import frc.robot.commands.autonomous.NeutralAutos;
import frc.robot.commands.autonomous.NeutralAutos.Routine;
import frc.robot.subsystems.aprilTagCam.AprilTagCam;
import frc.robot.subsystems.aprilTagCam.AprilTagCamConstants;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionSubsystem;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.objectDetection.GamePieceTracker;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.RotationTarget;
import frc.robot.subsystems.swerve.TunerConstants_Anemone;
import frc.robot.subsystems.swerve.TunerConstants_Mk4i;
import frc.robot.subsystems.swerve.TunerConstants_mk4n;
import frc.robot.subsystems.swerve.TunerConstants_mk5n;
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
    } else if (serialNumber.equals("88888")) {
      return Robot.DEV;
    } else if (serialNumber.equals("03223849")) {
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

  // private ObjectDetectionCam objDecCam;

  @SuppressWarnings("unused")
  private final BiConsumer<Runnable, Double> addPeriodic;

  private final CANBus rioCanbus = new CANBus("rio");
  private final CANBus canivoreCanbus = new CANBus("CAN_Network");

  private final StatusSignalCollection signalList = new StatusSignalCollection();

  private final ShooterSubsystem shooter;
  private final GroundIntakeRollerSubsystem groundIntakeRoller;
  private final GroundIntakeLinearExtensionSubsystem groundIntakeExtension;
  private ClimberSubsystem climber;
  private final IndexerSubsystem indexer;

  private final RobotVisualizer robotVisualizer;

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

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
                // as Red Team (win)
                if (currentShift == Shift.SHIFT_1 || currentShift == Shift.SHIFT_3) {
                  return timeRemaining >= lowerThreshold
                      || timeRemaining <= upperThreshold
                      || HubTracker.isActive();
                }
              } else {
                // as Blue Team (loss)
                if (currentShift == Shift.SHIFT_2 || currentShift == Shift.SHIFT_4) {
                  return timeRemaining >= lowerThreshold
                      || timeRemaining <= upperThreshold
                      || HubTracker.isActive();
                }
              }
            } else {
              // Blue Win
              if (!EagleUtil.isRedAlliance()) {
                // as Blue Team (win)
                if (currentShift == Shift.SHIFT_1 || currentShift == Shift.SHIFT_3) {
                  return timeRemaining >= lowerThreshold
                      || timeRemaining <= upperThreshold
                      || HubTracker.isActive();
                }
              } else {
                // as Red Team (loss)
                if (currentShift == Shift.SHIFT_2 || currentShift == Shift.SHIFT_4) {
                  return timeRemaining >= lowerThreshold
                      || timeRemaining <= upperThreshold
                      || HubTracker.isActive();
                }
              }
            }
            return HubTracker.isActive();
          });

  private AprilTagCam backRightCam;
  private AprilTagCam backLeftCam;
  private AprilTagCam frontRightCam;
  private AprilTagCam frontLeftCam;

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
        drivetrain = TunerConstants_mk5n.createDrivetrain();
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
            ShooterSubsystem.createReal(
                canivoreCanbus,
                rioCanbus,
                signalList,
                drivetrain.poseSupplier(),
                drivetrain::getVirtualTarget);
        climber = ClimberSubsystem.createDisabled();
        indexer = IndexerSubsystem.createReal(canivoreCanbus, rioCanbus, signalList);
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
        drivetrain = TunerConstants_mk5n.createDrivetrain();
        shooter =
            ShooterSubsystem.createSim(drivetrain.poseSupplier(), drivetrain::getVirtualTarget);
        climber = ClimberSubsystem.createSim();
        indexer = IndexerSubsystem.createSim();
        groundIntakeRoller = GroundIntakeRollerSubsystem.createSim();
        groundIntakeExtension = GroundIntakeLinearExtensionSubsystem.createSim();
        backRightCam =
            new AprilTagCam(
                AprilTagCamConstants.BACK_RIGHT_CAM,
                AprilTagCamConstants.BACK_RIGHT_CAM_LOCATION,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getCachedState().Pose,
                () -> drivetrain.getCachedState().Speeds);
        backLeftCam =
            new AprilTagCam(
                AprilTagCamConstants.BACK_LEFT_CAM,
                AprilTagCamConstants.BACK_LEFT_CAM_LOCATION,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getCachedState().Pose,
                () -> drivetrain.getCachedState().Speeds);

        frontRightCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_RIGHT_CAM,
                AprilTagCamConstants.FRONT_RIGHT_CAM_LOCATION,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getCachedState().Pose,
                () -> drivetrain.getCachedState().Speeds);

        frontLeftCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_LEFT_CAM,
                AprilTagCamConstants.FRONT_LEFT_CAM_LOCATION,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getCachedState().Pose,
                () -> drivetrain.getCachedState().Speeds);
        break;
      default:
        drivetrain = TunerConstants_mk5n.createDrivetrain();
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

    // objDecCam =
    //     new ObjectDetectionCam(
    //         "cam2026_01", ObjectDetectionConstants.robotToCam, () ->
    // drivetrain.getCachedState().Pose);

    configureBindings();
    configureAutonomous();

    drivetrain.setDefaultCommand(defualtDriveCommand);

    robotVisualizer = new RobotVisualizer(groundIntakeExtension);

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
    controller.leftBumper().onTrue(drivetrain.setRotationCommand(RotationTarget.NORMAL));
    // drivetrain.isOnBump.whileTrue(drivetrain.temporarilyDisableRotation());

    controller.rightTrigger().and(drivetrain.isInAllianceZone).whileTrue(shootHub());

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
    controller.rightTrigger().onFalse(stopShoot());

    controller.b().whileTrue(unStuck());

    controller.a().whileTrue(agitateGroundIntake());

    controller
        .y()
        .whileTrue(drivetrain.setShootingRange(true))
        .onFalse(drivetrain.setShootingRange(false));

    drivetrain.isInAllianceZone.onTrue(drivetrain.setRotationCommand(RotationTarget.HUB));
    drivetrain.isInAllianceZone.onTrue(shooter.preSpin());

    controller
        .rightBumper()
        .onTrue(drivetrain.setSlowMode(true))
        .onFalse(drivetrain.setSlowMode(false));

    controller.povDown().whileTrue(deployGroundIntake());
    controller.povDown().onFalse(groundIntakeRoller.stopIntake());

    controller.x().whileTrue(defenseMode());
    controller.start().onTrue(autoClimb());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureAutonomous() {
    NeutralAutos.configNeutralAutos(
        drivetrain, shooter, indexer, groundIntakeExtension, groundIntakeRoller, climber);
    autoChooser.addOption("Bump 1 Cycle Depot", new NeutralAutos(false, Routine.BUMP, false));
    autoChooser.addOption("Bump 1 Cycle Outpost", new NeutralAutos(true, Routine.BUMP, false));
    autoChooser.addOption("Bump 2 Cycle Depot", new NeutralAutos(false, Routine.BUMP, true));
    autoChooser.addOption("Bump 2 Cycle Outpost", new NeutralAutos(true, Routine.BUMP, true));
    autoChooser.addOption("Trench 1 Cycle Depot", new NeutralAutos(false, Routine.TRENCH, false));
    autoChooser.addOption("Trench 1 Cycle Outpost", new NeutralAutos(true, Routine.TRENCH, false));
    autoChooser.addOption("Trench 2 Cycle Depot", new NeutralAutos(false, Routine.TRENCH, true));
    autoChooser.addOption("Trench 2 Cycle Outpost", new NeutralAutos(true, Routine.TRENCH, true));
    autoChooser.addOption(
        "Depot 1 Cycle",
        new DepotPathAuto_1c(
            drivetrain, shooter, groundIntakeExtension, groundIntakeRoller, climber));
    SmartDashboard.putData("autonomous", autoChooser);
  }

  private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(
        0.660, // from left to right
        0.711, // from front to back
        0.127, // from floor to top of bumpers
        () -> drivetrain.getCachedState().Pose, // Supplier<Pose2d> of robot pose
        () ->
            ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getCachedState().Speeds,
                drivetrain.getCachedState().Pose.getRotation()));
    // Supplier<ChassisSpeeds> of field-centric chassis speeds

    // Register an intake to remove fuel from the field as a rectangular bounding box
    instance.registerIntake(
        0.350, 0.700, -0.330, 0.330); // robot-centric coordinates for bounding box

    instance.start();
  }

  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    startTime = HALUtil.getFPGATime();

    if (backRightCam != null) {
      backRightCam.updatePoseEstim();
      DogLog.log(
          "Loop Time/Robot Container/Back Right Cam", (HALUtil.getFPGATime() - startTime) / 1000);
    }

    startTime = HALUtil.getFPGATime();

    if (backLeftCam != null) {
      backLeftCam.updatePoseEstim();
      DogLog.log(
          "Loop Time/Robot Container/Back Left Cam", (HALUtil.getFPGATime() - startTime) / 1000);
    }

    startTime = HALUtil.getFPGATime();

    if (frontRightCam != null) {
      frontRightCam.updatePoseEstim();
      DogLog.log(
          "Loop Time/Robot Container/Front Right Cam", (HALUtil.getFPGATime() - startTime) / 1000);
    }

    startTime = HALUtil.getFPGATime();

    if (frontLeftCam != null) {
      frontLeftCam.updatePoseEstim();
      DogLog.log(
          "Loop Time/Robot Container/Front Left Cam", (HALUtil.getFPGATime() - startTime) / 1000);
    }

    // if (objDecCam != null) {
    //   objDecCam.updateDetection();
    // }

    DogLog.log(
        "Loop Time/Robot Container/objectDetection Cam",
        (HALUtil.getFPGATime() - startTime) / 1000);

    startTime = HALUtil.getFPGATime();

    if (RobotBase.isReal()) {
      signalList.refreshAll();
    }

    DogLog.log(
        "Loop Time/Robot Container/Refresh Signal List",
        (HALUtil.getFPGATime() - startTime) / 1000);

    startTime = HALUtil.getFPGATime();
    robotVisualizer.periodic();
    DogLog.log(
        "Loop Time/Robot Container/Robot Visualizer", (HALUtil.getFPGATime() - startTime) / 1000);

    startTime = HALUtil.getFPGATime();

    DogLog.log("Match Timer", DriverStation.getMatchTime());

    DogLog.log("Loop Time/Robot Container/Match Timer", (HALUtil.getFPGATime() - startTime) / 1000);

    startTime = HALUtil.getFPGATime();

    Optional<Pose2d> obj = GamePieceTracker.getGamePiece();

    DogLog.log(
        "Loop Time/Robot Container/Game Piece Tracker", (HALUtil.getFPGATime() - startTime) / 1000);

    DogLog.log("Hub Status/Is Active", isHubActive.getAsBoolean());

    if (obj.isPresent()) {
      DogLog.log("Object Detection/Fuel Pose", new Pose2d[] {obj.get()}); // ill forget it tommorow
    } else {
      DogLog.log("Object Detection/Fuel Pose", new Pose2d[0]); // ill forget it tommorow
    }
  }

  private Command disableHandler() {
    return Commands.sequence(
            shooter.stopShooter(),
            drivetrain.setRotationCommand(RotationTarget.NORMAL),
            climber.runVoltage(0))
        .ignoringDisable(true)
        .withName("Disabled");
  }

  public Command shootHub() {
    return Commands.parallel(
            drivetrain.setRotationCommand(RotationTarget.HUB),
            shooter.cruiseControl(),
            drivetrain.setSlowMode(0.5, 1),
            Commands.parallel(indexer.index(), EagleUtil.shootInSim(drivetrain))
                .onlyWhile(
                    shooter
                        .isAtGoalVelocity_Hub
                        .and(drivetrain.isFacingGoal)
                        .and(isHubActive)
                        .or(controller.leftTrigger()))
                .repeatedly())
        .withName("Shoot Hub");
  }

  public Command shootDepot() {
    return Commands.parallel(
            drivetrain.setRotationCommand(RotationTarget.PASSING_DEPOT_SIDE),
            shooter.cruiseControl(),
            drivetrain.setSlowMode(0.5, 1),
            Commands.parallel(indexer.index(), EagleUtil.shootInSim(drivetrain))
                .onlyWhile(
                    shooter
                        .isAtGoalVelocity_Passing
                        .and(drivetrain.isFacingGoalPassing)
                        .or(controller.leftTrigger()))
                .repeatedly())
        .withName("Shoot Depot Side");
  }

  public Command shootOutpost() {
    return Commands.parallel(
            drivetrain.setRotationCommand(RotationTarget.PASSING_OUTPOST_SIDE),
            shooter.cruiseControl(),
            drivetrain.setSlowMode(0.5, 1),
            Commands.parallel(indexer.index(), EagleUtil.shootInSim(drivetrain))
                .onlyWhile(
                    shooter
                        .isAtGoalVelocity_Passing
                        .and(drivetrain.isFacingGoalPassing)
                        .or(controller.leftTrigger()))
                .repeatedly())
        .withName("Shoot Outpost Side");
  }

  public Command unStuck() {
    return Commands.parallel(
            indexer.reverse(), groundIntakeRoller.reverseIntake(), groundIntakeExtension.extend())
        .withName("Unjam");
  }

  public Command deployGroundIntake() {
    return Commands.parallel(
            groundIntakeRoller.startIntake(),
            groundIntakeExtension.extend(),
            drivetrain.temporarilyDisableRotation().onlyWhile(controller.rightTrigger().negate()))
        .withName("Deploy Ground Intake");
  }

  public Command retractGroundIntake() {
    return Commands.parallel(
            groundIntakeRoller.stopIntake(),
            groundIntakeExtension.retract(),
            drivetrain.temporarilyDisableRotation().onlyWhile(controller.rightTrigger().negate()))
        .withName("Retract Ground Intake");
  }

  public Command defenseMode() {
    return Commands.parallel(
            drivetrain.swerveX(), groundIntakeExtension.retract(), groundIntakeRoller.stopIntake())
        .withName("Defense Mode");
  }

  public Command agitateGroundIntake() {
    return Commands.sequence(
            groundIntakeExtension.extend(),
            Commands.waitSeconds(.5),
            groundIntakeExtension.retract(),
            Commands.waitSeconds(.5),
            groundIntakeRoller.stopIntake())
        .repeatedly()
        .withName("Start? Ground Intake");
  }

  public Command stopShoot() {
    return Commands.parallel(
            drivetrain.setRotationCommand(RotationTarget.NORMAL),
            drivetrain.setSlowMode(false),
            shooter.stopShooter())
        .withName("Stop Shooting");
  }

  public Command autoClimb() {
    return Commands.sequence(
            Commands.parallel(
                drivetrain
                    .driveToPose(() -> EagleUtil.getClimbPose(drivetrain.getCachedState().Pose))
                    .until(
                        () ->
                            drivetrain
                                    .getCachedState()
                                    .Pose
                                    .getTranslation()
                                    .getDistance(
                                        EagleUtil.getClimbPose(drivetrain.getCachedState().Pose)
                                            .getTranslation())
                                < 0.10),
                groundIntakeRoller.stopIntake(),
                shooter.stopShooter(),
                Commands.sequence(
                    groundIntakeExtension.retract(),
                    Commands.waitSeconds(0.5),
                    climber.runPosition(ClimberConstants.PREP_CLIMB))),
            Commands.waitUntil(controller.start().debounce(0.1)),
            climber.runPosition(ClimberConstants.CLIMB_L1),
            Commands.waitUntil(controller.start().debounce(0.1)),
            climber.runPosition(ClimberConstants.PREP_CLIMB_MIDDLE),
            Commands.waitUntil(controller.start().debounce(0.1)),
            climber.runPosition(ClimberConstants.CLIMB_L2),
            Commands.waitUntil(controller.start().debounce(0.1)),
            climber.runPosition(ClimberConstants.PREP_CLIMB_MIDDLE),
            Commands.waitUntil(controller.start().debounce(0.1)),
            climber.runPosition(ClimberConstants.CLIMB_L3),
            Commands.waitUntil(controller.start().debounce(0.1)),
            Commands.parallel(indexer.index(), shooter.cruiseControl()))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
}
