package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionConstants;
import frc.robot.subsystems.groundIntakeLinearExtension.GroundIntakeLinearExtensionSubsystem;

public class RobotVisualizer extends SubsystemBase {
  private final GroundIntakeLinearExtensionSubsystem linearExt;

  public RobotVisualizer(GroundIntakeLinearExtensionSubsystem linearExt) {
    this.linearExt = linearExt;
  }

  @Override
  public void periodic() {
    double groundIntakeExtension =
        linearExt.getRotation()
            * 0.35
            / GroundIntakeLinearExtensionConstants.EXTENSION_ROTATION; // 35cm full out

    Pose3d groundIntakeRack = new Pose3d(groundIntakeExtension, 0, 0, new Rotation3d());
    Pose3d groundIntakePosition = new Pose3d(groundIntakeExtension, 0, 0, new Rotation3d());

    DogLog.log(
        "Robot Visualizer/Component Positions",
        new Pose3d[] {groundIntakeRack, groundIntakePosition});

<<<<<<< HEAD
    DogLog.log("extantion", groundIntakeExtension);
=======
    DogLog.log("get rotation", groundIntakeExtension);
>>>>>>> 6784cd7dc99444e4e167b535f3433bb507521240
  }
}
