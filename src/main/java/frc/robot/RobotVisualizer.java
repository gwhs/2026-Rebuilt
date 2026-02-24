package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionIOSim;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionSubsystem;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerIOSim;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerSubsystem;

public class RobotVisualizer extends SubsystemBase {
  private final GroundIntakeLinearExtensionSubsystem linearExt;

  public RobotVisualizer(
      GroundIntakeLinearExtensionSubsystem linearExt) {
    this.linearExt = linearExt;

  }

  @Override
  public void periodic() {
    double groundIntakeExtension = linearExt.getRotation() * 3.5; //35cm full out

    Pose3d groundIntakePosition =
        new Pose3d(
            -0.27, 0, 0.242, null);

    DogLog.log(
        "Robot Visualizer/Component Positions",
        new Pose3d[] {groundIntakePosition});
  }
}
