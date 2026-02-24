// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.EagleUtil;
import frc.robot.FieldConstants;
import frc.robot.subsystems.aprilTagCam.*;
import frc.robot.subsystems.objectDetection.GamePieceTracker;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.RotationTarget;
import frc.robot.subsystems.swerve.SwerveSubsystemConstants;
import java.util.Optional;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  /** Creates a new DriveCommand. */
  private final SwerveSubsystem drivetrain;

  private final CommandXboxController controller;

  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

  private final SlewRateLimiter angularVelocityLimiter;
  private final SlewRateLimiter xVelocityLimiter;
  private final SlewRateLimiter yVelocityLimiter;

  private final double maxSpeed = 4.5;
  private final double maxAngularSpeed = 3.0 * Math.PI;

  private final double deadband = 0.06;

  // TO-DO determine correct max accleration
  public final ProfiledPIDController robotHeadingController =
      new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(360, 720));
  public final PIDController shootingRangeDistance = new PIDController(0.3, 0, 0);

  private boolean resetLimiter = true;
  private boolean resetAutoRotate = true;

  public DriveCommand(SwerveSubsystem drivetrain, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.controller = controller;

    robotHeadingController.enableContinuousInput(-180, 180);

    angularVelocityLimiter = new SlewRateLimiter(1);
    xVelocityLimiter = new SlewRateLimiter(1);
    yVelocityLimiter = new SlewRateLimiter(1);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    double xInput = -controller.getLeftY();
    double yInput = -controller.getLeftX();
    double rotationalInput = -controller.getRightX();

    Pose2d currentRobotPose = drivetrain.getState().Pose;
    double currentRobotHeading = currentRobotPose.getRotation().getDegrees();
    Optional<Pose2d> currentGamePiecePose = GamePieceTracker.getGamePiece();

    xInput = MathUtil.applyDeadband(xInput, deadband);
    yInput = MathUtil.applyDeadband(yInput, deadband);
    rotationalInput = MathUtil.applyDeadband(rotationalInput, deadband);

    xInput = Math.pow(Math.abs(xInput), 1.5) * Math.signum(xInput);
    yInput = Math.pow(Math.abs(yInput), 1.5) * Math.signum(yInput);
    rotationalInput = Math.pow(Math.abs(rotationalInput), 1.5) * Math.signum(rotationalInput);

    boolean hasRotationInput = Math.abs(rotationalInput) > 0.1;
    if (drivetrain.getRotationTarget() != RotationTarget.NORMAL
        && !hasRotationInput
        && !drivetrain.getdisableAutoRotate()) {

      if (resetAutoRotate == true) {
        resetAutoRotate = false;
        robotHeadingController.reset(
            currentRobotHeading,
            Units.radiansToDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond));
      }

      robotHeadingController.setGoal(drivetrain.getGoalHeading());

      double pidOutput = robotHeadingController.calculate(currentRobotHeading);
      double setpointVelocity =
          robotHeadingController.getSetpoint().velocity / Units.radiansToDegrees(maxAngularSpeed);
      rotationalInput = MathUtil.clamp(pidOutput + setpointVelocity, -1, 1);

      DogLog.log("Drive Command/pidOutput", pidOutput);
      DogLog.log("Drive Command/rotational input", rotationalInput);
    } else {
      resetAutoRotate = true;
    }

    if (drivetrain.goingToShootingRange()) {
      SwerveDriveState state = drivetrain.getState();
      Translation2d hub =
          EagleUtil.isRedAlliance() ? FieldConstants.RED_HUB : FieldConstants.BLUE_HUB;
      Translation2d robotToHub = hub.minus(state.Pose.getTranslation());
      shootingRangeDistance.setSetpoint(SwerveSubsystemConstants.HUB_RADIUS);

      double shootingDistPIDOut =
          shootingRangeDistance.calculate(EagleUtil.getRobotTargetDistance(state.Pose, hub));
      Translation2d unitVec = robotToHub.div(robotToHub.getNorm());
      Translation2d goalVel = unitVec.times(shootingDistPIDOut);

      if (!EagleUtil.isRedAlliance()) {
        xInput -= goalVel.getX();
        yInput -= goalVel.getY();
      } else {
        xInput += goalVel.getX();
        yInput += goalVel.getY();
      }
    }

    if (drivetrain.isSlowMode()) {
      xInput = xInput * drivetrain.getTranslationSlowFactor();
      yInput = yInput * drivetrain.getTranslationSlowFactor();
      rotationalInput = rotationalInput * drivetrain.getRotationalSlowFactor();
    }

    if (drivetrain.isDrivingToFuel()
        && drivetrain.getDriveAssist()
        && currentGamePiecePose.isPresent()) {
      Pose2d coralRelativeToRobot = currentGamePiecePose.get().relativeTo(currentRobotPose);

      double errorY = coralRelativeToRobot.getY();

      double kP = 0.5507; // Change

      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        kP *= -1;
      }

      ChassisSpeeds assistedVectorRobotOriented = new ChassisSpeeds(0, errorY * kP, 0);
      DogLog.log("Intake Drive Assist/Assisted Robot Relative Vector", assistedVectorRobotOriented);

      ChassisSpeeds assistedVectorFieldOriented =
          ChassisSpeeds.fromRobotRelativeSpeeds(
              assistedVectorRobotOriented, currentRobotPose.getRotation());
      DogLog.log("Intake Drive Assist/Assisted Field Relative Vector", assistedVectorFieldOriented);

      xInput += -assistedVectorFieldOriented.vyMetersPerSecond;
      yInput += -assistedVectorFieldOriented.vxMetersPerSecond;
      rotationalInput += assistedVectorFieldOriented.omegaRadiansPerSecond;
    }

    if (drivetrain.isSlewRateLimitAcceleration()) {
      if (resetLimiter) {
        resetLimiter = false;
        xVelocityLimiter.reset(xInput);
        yVelocityLimiter.reset(yInput);
        angularVelocityLimiter.reset(rotationalInput);
      }

      xInput = xVelocityLimiter.calculate(xInput);
      yInput = yVelocityLimiter.calculate(yInput);
      rotationalInput = angularVelocityLimiter.calculate(rotationalInput);
    } else {
      resetLimiter = true;
    }

    double xVelocity = xInput * maxSpeed;
    double yVelocity = yInput * maxSpeed;
    double rotationalVelocity = rotationalInput * maxAngularSpeed;

    DogLog.log("Drive Command/xVelocity", xVelocity);
    DogLog.log("Drive Command/yVelocity", yVelocity);
    DogLog.log("Drive Command/rotationalVelocity)", rotationalVelocity);
    DogLog.log("Drive Command/Current Robot Heading (degree)", currentRobotHeading);
    DogLog.log("Drive Command/Auto Rotate goal (degree)", drivetrain.getGoalHeading());
    DogLog.log("Drive Command/auto heading Set Point", robotHeadingController.getSetpoint());
    DogLog.log("Drive Command/auto heading Goal", robotHeadingController.getGoal());
    DogLog.log("Drive Command/auto target", drivetrain.getRotationTarget());
    DogLog.log("Drive Command/driveAssist", drivetrain.getDriveAssist());
    drivetrain.setControl(
        fieldCentric
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(rotationalVelocity));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
