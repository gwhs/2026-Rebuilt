package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double ALLIANCE_ZONE_LINE_BLUE = 3.977894;
  public static final double ALLIANCE_ZONE_LINE_RED = 12.535154;
  public static double RED_DEPOTX = Units.inchesToMeters(2.014);
  public static double RED_DEPOTY = Units.inchesToMeters(6.704);
  public static double BLUE_DEPOTX = Units.inchesToMeters(9.604);
  public static double BLUE_DEPOTY = Units.inchesToMeters(7.602);
  public static double RED_OUTPOSTX = Units.inchesToMeters(1.845);
  public static double RED_OUTPOSTY = Units.inchesToMeters(1.976);
  public static double BLUE_OUTPOSTX = Units.inchesToMeters(9.535);
  public static double BLUE_OUTPOSTY = Units.inchesToMeters(0.888);
  public static Translation2d RED_DEPOT = new Translation2d(RED_DEPOTX, RED_DEPOTY);
  public static Translation2d BLUE_DEPOT = new Translation2d(BLUE_DEPOTX, BLUE_DEPOTY);
  public static Translation2d RED_OUTPOST = new Translation2d(RED_OUTPOSTX, RED_OUTPOSTY);
  public static Translation2d BLUE_OUTPOST = new Translation2d(BLUE_OUTPOSTX, BLUE_OUTPOSTY);
}
