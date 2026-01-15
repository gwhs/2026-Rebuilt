package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class EagleUtil {
    public static double getRobotTargetAngle(Pose2d robotpose, Translation2d target)
    {
        return target.minus(robotpose.getTranslation()).getAngle().getDegrees();
    }

    public static boolean isInAllianceZone(Pose2d robot)
    {
        return robot.getX() < 4.03;
    }

    public static boolean isInNeutralZone(Pose2d robot)
    {
        return (!(isInAllianceZone(robot) || isInOpponentZone(robot)));
    }

    public static boolean isInOpponentZone(Pose2d robot)
    {
        return robot.getX() > 12.01;
    }
    

    public static double getFlywheelSpeed (double distance)
    {
        return 0; //update
    }
}
