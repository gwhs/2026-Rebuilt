package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class EagleUtil {
    public static double radianToDegree(double radian)
    {
        return radian * (180 / Math.PI);
    }
    public static double degreeToRadian(double degree)
    {
        return degree * (Math.PI / 180);
    }
    public static double getRobotTargetAngle(Pose2d robotpose, Translation2d target)
    {
        double dx = robotpose.getX() - target.getX();
        double dy = robotpose.getY() - target.getY();
        double theta = radianToDegree(Math.atan(dy/dx));
        if (dx < 0)
        {
            theta = 180 - theta;
        }
        return theta;
    }
}
