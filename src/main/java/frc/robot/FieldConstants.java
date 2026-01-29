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

  public static final double ALLIANCE_ZONE_LINE_BLUE = 3.977894;
  public static final double ALLIANCE_ZONE_LINE_RED = 12.535154;

  public static final double FIELD_WIDTH = 8.042656;
  public static final double FIELD_HEIGHT = 16.513048;

  public static double fuelSpeed = 9.144;

  public static double BLUE_DEPOT_PASSING_X = 1.160;
  public static double BLUE_DEPOT_PASSING_Y = 6.896;
  public static double RED_OUTPOST_PASSING_X = 13.986;
  public static double RED_OUTPOST_PASSING_Y = 6.896;
  public static double BLUE_OUTPOST_PASSING_X = 1.160;
  public static double BLUE_OUTPOST_PASSING_Y = 1.116;
  public static double RED_DEPOT_PASSING_X = 13.986;
  public static double RED_DEPOT_PASSING_Y = 1.116;

  public static Translation2d RED_DEPOT_PASSING =
      new Translation2d(RED_DEPOT_PASSING_X, RED_DEPOT_PASSING_Y);
  public static Translation2d BLUE_DEPOT_PASSING =
      new Translation2d(BLUE_DEPOT_PASSING_X, BLUE_DEPOT_PASSING_Y);
  public static Translation2d RED_OUTPOST_PASSING =
      new Translation2d(RED_OUTPOST_PASSING_X, RED_OUTPOST_PASSING_Y);
  public static Translation2d BLUE_OUTPOST_PASSING =
      new Translation2d(BLUE_OUTPOST_PASSING_X, BLUE_OUTPOST_PASSING_Y);

  public static final double shooterAngleD = 60; // 72
  public static final double shooterAngleR = Math.PI / 3;
  public static final double hubHeight = 1.8;
  public static final double gravitationalAcc = 9.80665;
  public static final double shooterHeight = 0.635;
  public static final double motorToFlywheelGearRatio = 0.739;
  public static final double flywheelDiameter = 0.0762;
  public static final double robotCenterShooterDist = 0.254;
}
