package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
  public static double BLUE_HUBX = Units.inchesToMeters(182.11);
  public static double BLUE_HUBY = Units.inchesToMeters(158.84);
  public static Translation2d BLUE_HUB = new Translation2d(BLUE_HUBX, BLUE_HUBY);

  public static double RED_HUBX = Units.inchesToMeters(469.11);
  public static double RED_HUBY = Units.inchesToMeters(158.84);
  public static Translation2d RED_HUB = new Translation2d(RED_HUBX, RED_HUBY);

  public static double BUMP_WIDTH = 44.40;
  public static double ROBOT_DIAGIONAL_WIDTH = 36;
  public static double BLUE_BUMP_X1 = Units.inchesToMeters(156.60 - ROBOT_DIAGIONAL_WIDTH / 2);
  public static double BLUE_BUMP_X2 =
      Units.inchesToMeters(156.60 + BUMP_WIDTH + ROBOT_DIAGIONAL_WIDTH / 2);

  public static double RED_BUMP_X1 = Units.inchesToMeters(446.91 - ROBOT_DIAGIONAL_WIDTH / 2);
  public static double RED_BUMP_X2 =
      Units.inchesToMeters(446.91 + BUMP_WIDTH + ROBOT_DIAGIONAL_WIDTH / 2);

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

  public static final Pose2d BLUE_DEPOT_CLIMB =
      new Pose2d(1.565, 4.148, Rotation2d.kZero);

  public static final Pose2d BLUE_OUTPOT_CLIMB =
      new Pose2d(1.565, 3.290, Rotation2d.kZero);
  
  public static final Pose2d RED_DEPOT_CLIMB =
      new Pose2d(14.862, 3.840, Rotation2d.k180deg);

  public static final Pose2d RED_OUTPOT_CLIMB =
      new Pose2d(14.862, 4.740, Rotation2d.k180deg);

  public static final ArrayList<Pose2d> CLIMBPOSE = new ArrayList<>(Arrays.asList(BLUE_DEPOT_CLIMB,BLUE_OUTPOT_CLIMB,RED_DEPOT_CLIMB,RED_OUTPOT_CLIMB));
  
  public static Pose2d getClimbPose(Pose2d robotPose) {
    return robotPose.nearest(CLIMBPOSE);
    
    
  }

}
