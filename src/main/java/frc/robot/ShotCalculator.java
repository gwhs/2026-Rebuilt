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
    backBeltMap.put(1.1, 38.0);
    backBeltMap.put(1.35, 40.0);
    backBeltMap.put(2.82, 52.0);
    backBeltMap.put(3.94, 63.0);

    frontBeltMap.put(1.1, 43.0);
    frontBeltMap.put(1.35, 45.0);
    frontBeltMap.put(2.82, 57.0);
    frontBeltMap.put(3.94, 68.0);

    timeOfFlight.put(1.97, 1.26);
    timeOfFlight.put(2.560, 1.31);
    timeOfFlight.put(3.205, 1.80);
    timeOfFlight.put(1.046, 0.84);
    timeOfFlight.put(4.060, 1.65);
    timeOfFlight.put(5.260, 2.07);
    timeOfFlight.put(5.528, 2.04);
    timeOfFlight.put(3.010, 2.01);
    timeOfFlight.put(2.033, 1.57);
    timeOfFlight.put(1.601, 1.02);
    timeOfFlight.put(3.764, 1.48);
    timeOfFlight.put(4.778, 2.0);
    timeOfFlight.put(2.77, 1.99);
    timeOfFlight.put(7.080, 2.41);
    timeOfFlight.put(13.926, 4.44);
    timeOfFlight.put(10.001, 4.39);
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
