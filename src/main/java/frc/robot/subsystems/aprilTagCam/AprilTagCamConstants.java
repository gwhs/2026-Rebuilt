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

  public static final String FRONT_LEFT_CAMERA_COMP_NAME = "leftcam";
  public static final String FRONT_RIGHT_CAMERA_COMP_NAME = "rightcam";
  public static final String BACK_LEFT_CAMERA_COMP_NAME = "leftcam";
  public static final String BACK_RIGHT_CAMERA_COMP_NAME = "back_right_cam";
  public static final String ELEVATOR_CAMERA_COMP_NAME = "Arducam_OV9782_USB_Camera";
  public static final String FRONT_LEFT_CAMERA_DEV_NAME = "cam3";
  public static final String FRONT_RIGHT_CAMERA_DEV_NAME = "cam4";

  public static final Transform3d FRONT_RIGHT_CAMERA_LOCATION_DEV =
      new Transform3d(
          Units.inchesToMeters(9.524),
          Units.inchesToMeters(-11.185),
          Units.inchesToMeters(8.250),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(5)));

  public static final Transform3d FRONT_LEFT_CAMERA_LOCATION_DEV =
      new Transform3d(
          Units.inchesToMeters(9.524),
          Units.inchesToMeters(11.185),
          Units.inchesToMeters(8.250),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(-5)));

  public static final Transform3d FRONT_RIGHT_CAMERA_LOCATION_COMP =
      new Transform3d(
          Units.inchesToMeters(5.401),
          Units.inchesToMeters(-11.908),
          Units.inchesToMeters(7.033),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(18)));

  public static final Transform3d FRONT_LEFT_CAMERA_LOCATION_COMP =
      new Transform3d(
          Units.inchesToMeters(5.401),
          Units.inchesToMeters(11.908),
          Units.inchesToMeters(7.033),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(-18)));

  public static final Transform3d BACK_RIGHT_CAMERA_LOCATION_COMP =
      new Transform3d(
          Units.inchesToMeters(-8.752),
          Units.inchesToMeters(-10.953),
          Units.inchesToMeters(8.079),
          new Rotation3d(
              Units.degreesToRadians(0),
              Units.degreesToRadians(-15),
              Units.degreesToRadians(146.8)));

  public static final Transform3d ELEVATOR_CAMERA_LOCATION_COMP =
      new Transform3d(
          Units.inchesToMeters(-4.702),
          Units.inchesToMeters(6.142),
          Units.inchesToMeters(32.922),
          new Rotation3d(
              Units.degreesToRadians(2), Units.degreesToRadians(50), Units.degreesToRadians(180)));

  public static final double Z_TOLERANCE = 2.00;
  public static final double XY_TOLERANCE = 2.00;
  public static final double MAX_X_VALUE = 690.87;
  public static final double MAX_Y_VALUE = 317.00;
  public static final double SINGLE_APRILTAG_MAX_DISTANCE = 3.0;
  public static final double MULTI_APRILTAG_MAX_DISTANCE = 4.5;
  public static final double MAX_VELOCITY = 4;
  public static final double MAX_ROTATION = Math.PI;
}
