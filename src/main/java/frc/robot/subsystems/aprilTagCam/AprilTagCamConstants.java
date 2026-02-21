package frc.robot.subsystems.aprilTagCam;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class AprilTagCamConstants {
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.0, 1.0, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  public static final String BACK_RIGHT_CAM = "cam_back_right";
  public static final String BACK_LEFT_CAM = "cam_back_left";
  public static final String FRONT_LEFT_CAM = "cam_front_left";
  public static final String FRONT_RIGHT_CAM = "cam_front_right";

  public static final Transform3d BACK_RIGHT_CAM_LOCATION =
      new Transform3d(
          -0.302,
          -0.215,
          0.735,
          new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-15)));

  public static final Transform3d BACK_LEFT_CAM_LOCATION =
      new Transform3d(
          -0.302,
          0.215,
          0.735,
          new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(15)));

  public static final Transform3d FRONT_RIGHT_CAM_LOCATION =
      new Transform3d(
          0.309,
          -0.328,
          0.734,
          new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-100)));

  public static final Transform3d FRONT_LEFT_CAM_LOCATION =
      new Transform3d(
          0.309,
          0.328,
          0.735,
          new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(100)));

  public static final double Z_TOLERANCE = 2.00;
  public static final double XY_TOLERANCE = 2.00;
  public static final double MAX_X_VALUE = 690.87;
  public static final double MAX_Y_VALUE = 317.00;
  public static final double SINGLE_APRILTAG_MAX_DISTANCE = 3.0;
  public static final double MULTI_APRILTAG_MAX_DISTANCE = 4.5;
  public static final double MAX_VELOCITY = 4;
  public static final double MAX_ROTATION = Math.PI;
}
