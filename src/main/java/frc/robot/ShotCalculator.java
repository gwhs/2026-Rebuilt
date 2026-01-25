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
    shootVelocityMap.put(1.0, 3600.0);
    shootVelocityMap.put(1.2, 3800.0);
    shootVelocityMap.put(2.0, 4000.0);
    // ^^ placeholder/testing data, add real control points later
  }

  public static double getShootVelocity(double distance) {
    return shootVelocityMap.get(distance);
  }
}
