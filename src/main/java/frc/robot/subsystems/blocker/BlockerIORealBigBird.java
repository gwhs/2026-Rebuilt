// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.blocker;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Servo;

public class BlockerIORealBigBird implements BlockerIO {

  private final Servo servo1 = new Servo(BlockerConstants.SERVO_CHANNEL);

  public BlockerIORealBigBird() {}

  public double getMotor1Position() {

    return servo1.getAngle();
  }

  public void runPosition(double angle) {
    servo1.setAngle(angle);
  }

  public void periodic() {
    DogLog.log("Blocker/Servo 1 Angle", servo1.getAngle());
    DogLog.log("Blocker Running", servo1.getChannel());
  }
}
