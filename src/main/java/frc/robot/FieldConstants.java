package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static double BLUE_HUBX = Units.inchesToMeters(182.11);
  public static double BLUE_HUBY = Units.inchesToMeters(158.84);
  public static Translation2d BLUE_HUB = new Translation2d(BLUE_HUBX, BLUE_HUBY);

  public static double RED_HUBX = Units.inchesToMeters(469.11);
  public static double RED_HUBY = Units.inchesToMeters(158.84);
  public static Translation2d RED_HUB = new Translation2d(RED_HUBX, RED_HUBY);

  public static double BUMP_WIDTH = 30;
  public static double BLUE_BUMP_X1 = Units.inchesToMeters(446.91);
  public static double BLUE_BUMP_X2 = Units.inchesToMeters(482.22 + BUMP_WIDTH);

  public static double RED_BUMP_X1 = Units.inchesToMeters(121.30);
  public static double RED_BUMP_X2 = Units.inchesToMeters(156.61 + BUMP_WIDTH);

  public static final double ALLIANCE_ZONE_LINE_BLUE = 3.977894;
  public static final double ALLIANCE_ZONE_LINE_RED = 12.535154;
}
