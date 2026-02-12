package frc.robot.subsystems.aprilTagCam;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class AprilTagCamConstants {
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.0, 1.0, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  public static final String TEST_CAM_ONE = "cam5";

  public static final Transform3d TEST_CAM_ONE_LOCATION =
      new Transform3d(-0.124, 0.306, -0.17944, new Rotation3d(0, 0, 0));

  //           public static final Transform3d TEST_CAM_ONE_LOCATION =
  //   new Transform3d(
  //       Units.inchesToMeters(-0.124),
  //       Units.inchesToMeters(0.306),
  //       Units.inchesToMeters(-0.17944),
  //       new Rotation3d(
  //           Units.degreesToRadians(2), Units.degreesToRadians(50), Units.degreesToRadians(180)));

  public static final double Z_TOLERANCE = 2.00;
  public static final double XY_TOLERANCE = 2.00;
  public static final double MAX_X_VALUE = 690.87;
  public static final double MAX_Y_VALUE = 317.00;
  public static final double SINGLE_APRILTAG_MAX_DISTANCE = 3.0;
  public static final double MULTI_APRILTAG_MAX_DISTANCE = 4.5;
  public static final double MAX_VELOCITY = 4;
  public static final double MAX_ROTATION = Math.PI;
}
