// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ShotCalculator {

  private static final InterpolatingDoubleTreeMap shootVelocityMap =
      new InterpolatingDoubleTreeMap();

  static {
    shootVelocityMap.put(1.0, 36.0);
    shootVelocityMap.put(1.2, 40.0);
    shootVelocityMap.put(2.0, 60.0);
    // ^^ placeholder/testing data, add real control points later
  }

  public static double getShootVelocity(double distance) {
    return shootVelocityMap.get(distance);
  }
}
