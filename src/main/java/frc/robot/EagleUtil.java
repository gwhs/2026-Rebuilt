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
        double dx = target.getX() - robotpose.getX();
        double dy = target.getY() - robotpose.getY();
        double theta = radianToDegree(Math.atan(dy/dx));
        if (dx < 0)
        {
            theta = 180 - theta;
        }
        if (dy < 0)
        {
            theta += 180;
        }
        return theta;
    }
}
