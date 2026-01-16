package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double ALLIANCE_ZONE_LINE_BLUE = 3.977894;
  public static final double ALLIANCE_ZONE_LINE_RED = 12.535154;
  public static double RED_DEPOT_PASSING_X = Units.inchesToMeters(2.014);
  public static double RED_DEPOT_PASSING_Y = Units.inchesToMeters(6.704);
  public static double BLUE_DEPOT_PASSING_X = Units.inchesToMeters(9.604);
  public static double BLUE_DEPOT_PASSING_Y = Units.inchesToMeters(7.602);
  public static double RED_OUTPOST_PASSING_X = Units.inchesToMeters(1.845);
  public static double RED_OUTPOST_PASSING_Y = Units.inchesToMeters(1.976);
  public static double BLUE_OUTPOST_PASSING_X = Units.inchesToMeters(9.535);
  public static double BLUE_OUTPOST_PASSING_Y = Units.inchesToMeters(0.888);
  public static Translation2d RED_DEPOT_PASSING =
      new Translation2d(RED_DEPOT_PASSING_X, RED_DEPOT_PASSING_Y);
  public static Translation2d BLUE_DEPOT_PASSING =
      new Translation2d(BLUE_DEPOT_PASSING_X, BLUE_DEPOT_PASSING_Y);
  public static Translation2d RED_OUTPOST_PASSING =
      new Translation2d(RED_OUTPOST_PASSING_X, RED_OUTPOST_PASSING_Y);
  public static Translation2d BLUE_OUTPOST_PASSING =
      new Translation2d(BLUE_OUTPOST_PASSING_X, BLUE_OUTPOST_PASSING_Y);
}
