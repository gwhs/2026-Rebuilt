package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerSwerveDrivetrain;
import frc.robot.subsystems.aprilTagCam.AprilTagHelp;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private final TalonFX[] driveMotors = new TalonFX[4];
  private final TalonFX[] steerMotors = new TalonFX[4];
  private final CANcoder[] encoders = new CANcoder[4];
  private final Pigeon2 gyro;

  public Trigger IS_ALIGNING_TO_POSE =
      new Trigger(
          () -> {
            if (this.getCurrentCommand() != null) {
              return this.getCurrentCommand().getName().equals("AlignToPose");
            } else {
              return false;
            }
          });

  public Constraints constraints = new TrapezoidProfile.Constraints(3, 2);
  public ProfiledPIDController PID_X = new ProfiledPIDController(3.0, 0, 0, constraints);
  public ProfiledPIDController PID_Y = new ProfiledPIDController(3.0, 0, 0, constraints);

  public PIDController PID_Rotation = new PIDController(0.05, 0, 0);
  public Trigger IS_AT_TARGET_POSE =
      new Trigger(() -> PID_X.atSetpoint() && PID_Y.atSetpoint() && PID_Rotation.atSetpoint());

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.73);

  public double getDriveBaseRadius() {
    Translation2d[] moduleLocations = getModuleLocations();
    return Math.max(
        Math.max(
            Math.hypot(moduleLocations[0].getX(), moduleLocations[0].getY()),
            Math.hypot(moduleLocations[1].getX(), moduleLocations[1].getY())),
        Math.max(
            Math.hypot(moduleLocations[2].getX(), moduleLocations[1].getY()),
            Math.hypot(moduleLocations[3].getX(), moduleLocations[3].getY())));
  }

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    PID_Rotation.setTolerance(0.5);
    PID_Rotation.enableContinuousInput(-180, 180);
    PID_Y.setTolerance(0.02);
    PID_X.setTolerance(0.02);
    configureAutoBuilder();

    for (int i = 0; i < 4; i++) {
      SwerveModule<TalonFX, TalonFX, CANcoder> module = getModule(i);
      driveMotors[i] = module.getDriveMotor();
      steerMotors[i] = module.getSteerMotor();
      encoders[i] = module.getEncoder();
    }
    gyro = getPigeon2();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();

    for (int i = 0; i < 4; i++) {
      SwerveModule<TalonFX, TalonFX, CANcoder> module = getModule(i);
      driveMotors[i] = module.getDriveMotor();
      steerMotors[i] = module.getSteerMotor();
      encoders[i] = module.getEncoder();
    }
    gyro = getPigeon2();
  }

  /**
   * @param targetPose the pose to go to
   */
  public void goToPoseWithPID(Pose2d targetPose) {
    ChassisSpeeds currentSpeed =
        ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, getRotation());

    double predicted_X = (targetPose.getX() - getPose().getX()) * 0.3 + getPose().getX();
    double predicted_Y = (targetPose.getY() - getPose().getY()) * 0.3 + getPose().getY();

    PID_X.reset(predicted_X, currentSpeed.vxMetersPerSecond * 0.4);
    PID_Y.reset(predicted_Y, currentSpeed.vyMetersPerSecond * 0.4);
    PID_X.setGoal(targetPose.getX());
    PID_Y.setGoal(targetPose.getY());
    PID_Rotation.setSetpoint(targetPose.getRotation().getDegrees());
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();

    for (int i = 0; i < 4; i++) {
      SwerveModule<TalonFX, TalonFX, CANcoder> module = getModule(i);
      driveMotors[i] = module.getDriveMotor();
      steerMotors[i] = module.getSteerMotor();
      encoders[i] = module.getEncoder();
    }
    gyro = getPigeon2();
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
              new PIDConstants(5, 0, 0),
              // PID constants for rotation
              new PIDConstants(5, 0, 0)),
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

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param requestSupplier Function returning the request to apply
   * @param requestSupplier Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
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
    DogLog.log("Command Swerve DriveTrain/is Aligning to pose", IS_ALIGNING_TO_POSE.getAsBoolean());
    DogLog.log("Command Swerve DriveTrain/is at target pose", IS_AT_TARGET_POSE.getAsBoolean());

    DogLog.log("Swerve/current X setpoint", PID_X.getSetpoint().position);
    DogLog.log("Swerve/current Y setpoint", PID_Y.getSetpoint().position);

    DogLog.log(
        "Swerve/current setpoint",
        new Pose2d(
            PID_X.getSetpoint().position,
            PID_Y.getSetpoint().position,
            Rotation2d.fromDegrees(PID_Rotation.getSetpoint())));

    DogLog.log("Swerve/Front Left Drive Motor Connected", driveMotors[0].isConnected());
    DogLog.log("Swerve/Front Left Steer Motor Connected", steerMotors[0].isConnected());
    DogLog.log("Swerve/Front Left CANcoder Connected", encoders[0].isConnected());

    DogLog.log("Swerve/Front Right Drive Motor Connected", driveMotors[1].isConnected());
    DogLog.log("Swerve/Front Right Steer Motor Connected", steerMotors[1].isConnected());
    DogLog.log("Swerve/Front Right CANcoder Connected", encoders[1].isConnected());

    DogLog.log("Swerve/Back Left Drive Motor Connected", driveMotors[2].isConnected());
    DogLog.log("Swerve/Back Left Steer Motor Connected", steerMotors[2].isConnected());
    DogLog.log("Swerve/Back Left CANcoder Connected", encoders[2].isConnected());

    DogLog.log("Swerve/Back Right Drive Motor Connected", driveMotors[3].isConnected());
    DogLog.log("Swerve/Back Right Steer Motor Connected", steerMotors[3].isConnected());
    DogLog.log("Swerve/Back Right CANcoder Connected", encoders[3].isConnected());

    DogLog.log("Swerve/Pigeon Connected", gyro.isConnected());
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

  /**
   * @param helper the apriltag helper object NOTE: Look at the AprilTagCam stuff if you want to
   *     know about this
   */
  public void addVisionMeasurent(AprilTagHelp helper) {

    Pose2d pos = helper.pos;
    Matrix<N3, N1> sd = helper.sd;
    double timestamp = helper.timestamp;

    super.addVisionMeasurement(pos, timestamp, sd);
  }

  /**
   * @return the Pose2d of the robot
   */
  public Pose2d getPose() {
    return getState().Pose;
  }

  public Pose2d getPose(double timeSeconds) {
    Pose2d currPose = this.getPose();
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

  /**
   * @return the Rotation2d of the robot NOTE: does not return the angle as a double. It returns it
   *     as Rotation2d object
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * @return Returns the position of each module in radians.
   */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] =
          Units.rotationsToRadians(
              getModule(i).getDriveMotor().getPosition(true).getValueAsDouble()
                  / 6.746031746031747);
    }
    return values;
  }

  /**
   * @param speeds the chassis speeds
   */
  public void runVelocity(ChassisSpeeds speeds) {
    setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
  }

  public Command driveBackward(double velocity) {
    return this.run(
            () ->
                this.setControl(
                    robotCentricDrive
                        .withVelocityX(-velocity)
                        .withVelocityY(0)
                        .withRotationalRate(0)))
        .finallyDo(
            () ->
                this.setControl(
                    robotCentricDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)));
  }
}
