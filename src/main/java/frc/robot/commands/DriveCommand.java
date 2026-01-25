// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.RotationTarget;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  /** Creates a new DriveCommand. */
  private final SwerveSubsystem drivetrain;

  private final CommandXboxController controller;

  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

  private final double maxSpeed = 4.5;
  private final double maxAngularSpeed = 2.5 * Math.PI;

  private final double deadband = 0.1;

  public final PIDController robotHeadingController = new PIDController(0.04, 0, 0);

  public DriveCommand(SwerveSubsystem drivetrain, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.controller = controller;

    robotHeadingController.enableContinuousInput(-180, 180);

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

    boolean hasRotationInput = Math.abs(rotationalInput) > 0.1;
    if (drivetrain.getRotationTarget() != RotationTarget.NORMAL
        && !hasRotationInput
        && !drivetrain.getdisableAutoRotate()) {

      double currentRobotHeading = drivetrain.getState().Pose.getRotation().getDegrees();

      robotHeadingController.setSetpoint(drivetrain.getGoalHeading());

      double pidOutput = robotHeadingController.calculate(currentRobotHeading);
      
      rotationalInput = MathUtil.clamp(pidOutput, -0.5, 0.5);

      DogLog.log("Drive Command/Auto Rotate PID output", pidOutput);
      DogLog.log("Drive Command/Auto Rotate goal (degree)", drivetrain.getGoalHeading());
      DogLog.log("Drive Command/Current Robot Heading (degree)", currentRobotHeading);
    }

    if (drivetrain.isSlowMode()) {
      xInput = xInput * drivetrain.getTranslationSlowFactor();
      yInput = yInput * drivetrain.getTranslationSlowFactor();
      rotationalInput = rotationalInput * drivetrain.getRotationalSlowFactor();
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
