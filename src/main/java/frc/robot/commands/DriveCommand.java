// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.EagleUtil;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.RotationTarget;
import frc.robot.subsystems.swerve.SwerveSubsystemConstants;

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
  private final double maxAngularSpeed =
      3.0 * Math.PI; // one rotation per second. need to test on real robot

  private final double deadband = 0.1;

  // TO-DO determine correct max accleration
  public final ProfiledPIDController robotHeadingController =
      new ProfiledPIDController(0.15, 0, 0.02, new TrapezoidProfile.Constraints(1, 10));
  public final PIDController shootingRangeDistance = new PIDController(0.3, 0, 0);

  private boolean resetLimiter = true;

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

      double currentRobotHeading = drivetrain.getState().Pose.getRotation().getDegrees();

      robotHeadingController.setGoal(drivetrain.getGoalHeading());

      double pidOutput = robotHeadingController.calculate(currentRobotHeading);

      rotationalInput = MathUtil.clamp(pidOutput, -1, 1);

      DogLog.log("Drive Command/Auto Rotate PID output", pidOutput);
      DogLog.log("Drive Command/Auto Rotate goal (degree)", drivetrain.getGoalHeading());
      DogLog.log("Drive Command/Current Robot Heading (degree)", currentRobotHeading);
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
