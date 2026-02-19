// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ShotCalculator {

  private static final InterpolatingDoubleTreeMap frontBeltMap = new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap backBeltMap = new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap timeOfFlight = new InterpolatingDoubleTreeMap();

  static {
    frontBeltMap.put(1.0, 36.0);
    frontBeltMap.put(1.2, 40.0);
    frontBeltMap.put(2.0, 60.0);

    backBeltMap.put(1.0, 36.0);
    backBeltMap.put(1.2, 40.0);
    backBeltMap.put(2.0, 60.0);

    timeOfFlight.put(1.0, 1.0);
    timeOfFlight.put(1.2, 1.5);
    timeOfFlight.put(2.0, 2.0);
    // ^^ placeholder/testing data, add real control points later
  }

  public static double getFrontVelocity(double distance) {
    return frontBeltMap.get(distance);
  }

  public static double getBackVelocity(double distance) {
    return backBeltMap.get(distance);
  }

  public static double getTimeOfFlight(double distance) {
    return timeOfFlight.get(distance);
  }
}
