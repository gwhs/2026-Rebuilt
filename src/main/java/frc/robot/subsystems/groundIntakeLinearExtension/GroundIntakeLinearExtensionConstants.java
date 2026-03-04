package frc.robot.subsystems.groundIntakeLinearExtension;

import edu.wpi.first.math.util.Units;

public class GroundIntakeLinearExtensionConstants {
  public static final int MOTOR_ID = 32;
  public static final int PIVOT_MOTOR_ID = 33;

  public static final double MAX_VELOCITY = 1.2; // THINGSTODO
  public static final double MAX_ACCELERATION = 3.0; // TODO

  public static final double MAX_ROTATION = Units.degreesToRotations(125);
  public static final double MIN_ROTATION = 0;

  public static final double EXTENSION_ROTATION = Units.degreesToRadians(125);
  public static final double RETRACT_ROTATION = 0;
}
