package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.EagleUtil;
import frc.robot.FieldConstants;
import frc.robot.commands.AlignToPose;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class SwerveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements Subsystem {

  public enum RotationTarget {
    NORMAL,
    FORTY_FIVE,
    PASSING_DEPOT_SIDE,
    PASSING_OUTPOST_SIDE,
    TOWER,
    HUB,
    TST,
  }

  private Alert frontLeftDriveConnectedAlert =
      new Alert("Front left drive motor is not connected!", AlertType.kError);
  private Alert frontLeftTurnConnectedAlert =
      new Alert("Front left turn motor is not connected!", AlertType.kError);
  private Alert frontLeftEncoderConnectedAlert =
      new Alert("Front left encoder is not connected!", AlertType.kError);
  private Alert backLeftDriveConnectedAlert =
      new Alert("back left drive motor is not connected!", AlertType.kError);
  private Alert backLeftTurnConnectedAlert =
      new Alert("back left turn motor is not connected!", AlertType.kError);
  private Alert backleftEncoderConnectedAlert =
      new Alert("Back left encoder is not connected!", AlertType.kError);
  private Alert frontRightDriveConnectedAlert =
      new Alert("Front right drive motor is not connected!", AlertType.kError);
  private Alert frontRightTurnConnectedAlert =
      new Alert("Front right turn motor is not connected!", AlertType.kError);
  private Alert frontrightEncoderConnectedAlert =
      new Alert("Front right encoder is not connected!", AlertType.kError);
  private Alert backRightDriveConnectedAlert =
      new Alert("Back right drive motor is not connected!", AlertType.kError);
  private Alert backRightTurnConnectedAlert =
      new Alert("Back right turn motor is not connected!", AlertType.kError);
  private Alert backRightEncoderConnectedAlert =
      new Alert("Back right encoder is not connected!", AlertType.kError);
  private Alert pigeonConnectedAlert = new Alert("Pigeon is not connected", AlertType.kError);

  private TalonFX frontLeftDrive = this.getModule(0).getDriveMotor();
  private TalonFX frontLeftTurn = this.getModule(0).getSteerMotor();
  private CANcoder frontLeftEncoder = this.getModule(0).getEncoder();
  private TalonFX frontRightDrive = this.getModule(1).getDriveMotor();
  private TalonFX frontRightTurn = this.getModule(1).getSteerMotor();
  private CANcoder frontRightEncoder = this.getModule(1).getEncoder();
  private TalonFX backLeftDrive = this.getModule(2).getDriveMotor();
  private TalonFX backLeftTurn = this.getModule(2).getSteerMotor();
  private CANcoder backLeftEncoder = this.getModule(2).getEncoder();
  private TalonFX backRightDrive = this.getModule(3).getDriveMotor();
  private TalonFX backRightTurn = this.getModule(3).getSteerMotor();
  private CANcoder backRightEncoder = this.getModule(3).getEncoder();
  private Pigeon2 pigeon = this.getPigeon2();

  private boolean disableAutoRotate = false;
  private RotationTarget rotationTarget = RotationTarget.NORMAL;
  private CommandXboxController controller;
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  private final Telemetry logger = new Telemetry();

  public Trigger isInAllianceZone = new Trigger(() -> EagleUtil.isInAllianceZone(getState().Pose));
  public Trigger isInOpponentAllianceZone =
      new Trigger(() -> EagleUtil.isInOpponentAllianceZone(getState().Pose));
  public Trigger isInNeutralZone = new Trigger(() -> EagleUtil.isInNeutralZone(getState().Pose));

  public Trigger isOnDepotSide = new Trigger(() -> EagleUtil.isOnDepotSide(getState().Pose));
  public Trigger isOnOutpostSide = new Trigger(() -> EagleUtil.isOnOutpostSide(getState().Pose));

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

  private double translationSlowFactor = 1;
  private double rotationalSlowFactor = 1;
  private boolean slowMode = false;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public SwerveSubsystem(
      CommandXboxController controller,
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    this.controller = controller;

    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
    registerTelemetry(logger::telemeterize);
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
    DogLog.log("Current Zone/In Alliance Zone", isInAllianceZone.getAsBoolean());
    DogLog.log("Current Zone/In Opponent Alliance Zone", isInOpponentAllianceZone.getAsBoolean());
    DogLog.log("Current Zone/In Neutral Zone", isInNeutralZone.getAsBoolean());
    DogLog.log("Current Zone/On Depot Side", isOnDepotSide.getAsBoolean());
    DogLog.log("Current Zone/On Outpost Side", isOnOutpostSide.getAsBoolean());
    DogLog.log("Intake Drive Assist/Is Driving Toward Fuel", isDrivingToFuel());

    frontLeftDriveConnectedAlert.set(!frontLeftDrive.isConnected());
    frontLeftTurnConnectedAlert.set(!frontLeftTurn.isConnected());
    backLeftDriveConnectedAlert.set(!backLeftDrive.isConnected());
    backLeftTurnConnectedAlert.set(!backLeftTurn.isConnected());
    frontRightDriveConnectedAlert.set(!frontRightDrive.isConnected());
    frontRightTurnConnectedAlert.set(!frontRightTurn.isConnected());
    backRightDriveConnectedAlert.set(!backRightDrive.isConnected());
    backRightTurnConnectedAlert.set(!backRightTurn.isConnected());
    frontLeftEncoderConnectedAlert.set(!frontLeftEncoder.isConnected());
    backleftEncoderConnectedAlert.set(!backLeftEncoder.isConnected());
    frontrightEncoderConnectedAlert.set(!frontRightEncoder.isConnected());
    backRightEncoderConnectedAlert.set(!backRightEncoder.isConnected());
    pigeonConnectedAlert.set(!pigeon.isConnected());
  }

  private double defualtSlowFactor = 0.25;

  public Command setSlowMode(boolean enable) {
    return Commands.runOnce(
        () -> {
          this.slowMode = enable;
          this.translationSlowFactor = defualtSlowFactor;
          this.rotationalSlowFactor = defualtSlowFactor;
        });
  }

  public Command setSlowMode(double translationSlowFactor, double rotationalSlowFactor) {
    return Commands.runOnce(
        () -> {
          this.slowMode = true;
          this.translationSlowFactor = MathUtil.clamp(translationSlowFactor, 0, 1);
          this.rotationalSlowFactor = MathUtil.clamp(rotationalSlowFactor, 0, 1);
        });
  }

  public double getTranslationSlowFactor() {
    return translationSlowFactor;
  }

  public double getRotationalSlowFactor() {
    return rotationalSlowFactor;
  }

  public boolean isSlowMode() {
    return slowMode;
  }

  public boolean getdisableAutoRotate() {
    return disableAutoRotate;
  }

  public Command setRotationCommand(RotationTarget rotationTarget) {
    return Commands.runOnce(
        () -> {
          this.rotationTarget = rotationTarget;
        });
  }

  public RotationTarget getRotationTarget() {
    return this.rotationTarget;
  }

  public double getGoalHeading() {
    switch (this.rotationTarget) {
      case NORMAL:
        return 0;
      case PASSING_DEPOT_SIDE:
        if (EagleUtil.isRedAlliance()) {
          return EagleUtil.getRobotTargetAngle(getState().Pose, FieldConstants.RED_DEPOT_PASSING);
        } else {
          return EagleUtil.getRobotTargetAngle(getState().Pose, FieldConstants.BLUE_DEPOT_PASSING);
        }
      case PASSING_OUTPOST_SIDE:
        if (EagleUtil.isRedAlliance()) {
          return EagleUtil.getRobotTargetAngle(getState().Pose, FieldConstants.RED_OUTPOST_PASSING);
        } else {
          return EagleUtil.getRobotTargetAngle(
              getState().Pose, FieldConstants.BLUE_OUTPOST_PASSING);
        }
      case TOWER:
        return 0;
      case HUB:
        return EagleUtil.getRotationalHub(getState().Pose);
      case TST: // test case for shoot on move
        return EagleUtil.getRobotTargetAngle(
            getState().Pose,
            EagleUtil.calcAimpoint(
                getState().Pose, getPose(0.2), FieldConstants.RED_HUB, getState().Speeds));
      default:
        return 0;
    }
  }

  public boolean isDrivingToFuel() {
    ChassisSpeeds currRobotSpeed = getState().Speeds;
    return currRobotSpeed.vxMetersPerSecond > 0.1;
  }

  public Pose2d getPose(double timeSeconds) {
    Pose2d currPose = getState().Pose;
    ChassisSpeeds speeds = getState().Speeds;
    double velocityX = speeds.vxMetersPerSecond;
    double velocityY = speeds.vyMetersPerSecond;

    double transformX = timeSeconds * velocityX;
    double transformY = timeSeconds * velocityY;
    Rotation2d transformRotation = new Rotation2d(timeSeconds * speeds.omegaRadiansPerSecond);
    Transform2d transformPose = new Transform2d(transformX, transformY, transformRotation);
    Pose2d predictedPose = currPose.plus(transformPose);

    DogLog.log("Predicted Pose", predictedPose);

    return predictedPose;
  }

  public Command driveToPose(Supplier<Pose2d> pose) {
    return new AlignToPose(
        pose,
        this,
        () -> {
          return 0.0;
        },
        this.controller);
  }

  /*
   * the idea behind this command is that it:
   * 1. saves current rotation target,
   * 2. resets the rotation target to normal
   * 3. does whatever inbetween thing needs to be done while we aren't aligning
   * 4. set the target back to the previous target.
   */
  public Command temporarilyDisableRotation() {
    return Commands.run(
            () -> {
              this.disableAutoRotate = true;
            })
        .finallyDo(
            () -> {
              this.disableAutoRotate = false;
            });
  }
}
