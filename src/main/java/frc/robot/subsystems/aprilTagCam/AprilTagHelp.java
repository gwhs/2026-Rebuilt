package frc.robot.subsystems.aprilTagCam;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class AprilTagHelp {

  public Matrix<N3, N1> sd;
  public Pose2d pos;
  public double timestamp;

  public AprilTagHelp(Pose2d pos, double timestamp, Matrix<N3, N1> sd) {
    this.sd = sd;
    this.pos = pos;
    this.timestamp = timestamp;
  }

  public void update(Pose2d pos, double timestamp, Matrix<N3, N1> sd) {
    this.sd = sd;
    this.pos = pos;
    this.timestamp = timestamp;
  }

  public Matrix<N3, N1> getSD() {
    return sd;
  }
}
