package frc.robot.subsystems.groundIntakePivot;

public class GroundIntakePivotConstants {
  public static final int GROUND_INTAKE_PIVOT_MOTOR_ID = 15;

  public static final int GROUND_INTAKE_PIVOT_ENCODER_ID = 20;
  public static final double MAX_VELOCITY = 1.2; // rotation per second
  public static final double MAX_ACCELERATION = 3.0; // rotation per second per second
  public static final double GROUND_INTAKE_PIVOT_GEAR_RATIO = 68.0 / 12 * 84 / 20 * 48 / 18; // 64

  // Position constants: in degrees
  public static final double L1_PREP_POSITION = -80.320;
  public static final double L2_PREP_POSITION = 133;
  public static final double L3_PREP_POSITION = 124.565;
  public static final double L4_PREP_POSITION = 120;

  public static final double GROUND_INTAKE_PIVOT_UPPER_BOUND = 270;
  public static final double GROUND_INTAKE_PIVOT_LOWER_BOUND = -170;

  public static final double GROUND_INTAKE_PIVOT_INTAKE_ANGLE = -72;
  public static final double GROUND_INTAKE_PIVOT_INTAKE_ANGLE_BACKUP = -65;
  public static final double GROUND_INTAKE_PIVOT_STOW_ANGLE = -90.0;

  public static final double PRE_DEALGAE_ANGLE = 155;
  public static final double DEALGAE_ANGLE = 170;

  public static final double MAGNET_OFFSET_DEGREES = -269.82;

  public static final double PREP_CLIMB_ANGLE = -170;
  public static final double CLIMB_ANGLE = -10;
}

