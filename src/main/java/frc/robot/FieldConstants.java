package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static double BLUE_HUBX = Units.inchesToMeters(182.11);
  public static double BLUE_HUBY = Units.inchesToMeters(158.84);
  static Translation2d BLUE_HUB = new Translation2d(BLUE_HUBX, BLUE_HUBY);

  public static double RED_HUBX = Units.inchesToMeters(469.11);
  public static double RED_HUBY = Units.inchesToMeters(158.84);
  static Translation2d RED_HUB = new Translation2d(RED_HUBX, RED_HUBY);
}
