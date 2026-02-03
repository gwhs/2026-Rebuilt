package frc.robot.subsystems.led;

import com.ctre.phoenix6.CANBus;

class LedConstants {
  public static int CANdleID = 23;
  public static CANBus CANdleCANBus = new CANBus("rio");
  public static int IndexMax = 100; // TODO: change to actual size
}
