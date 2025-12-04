package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.List;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private double prevTime = HALUtil.getFPGATime();
  private final GcStatsCollector gcStatsCollector = new GcStatsCollector();

  public Robot() {
    // Setup DogLog
    DogLog.setOptions(
        new DogLogOptions().withNtPublish(true).withCaptureNt(true).withCaptureDs(true));
    DogLog.setPdh(new PowerDistribution());

    m_robotContainer = new RobotContainer(this::addPeriodic);

    LiveWindow.disableAllTelemetry();

    DriverStation.silenceJoystickConnectionWarning(true);

    NetworkTableInstance.getDefault()
        .getStringTopic("/Metadata/Branch")
        .publish()
        .set(BuildConstants.GIT_BRANCH);
    NetworkTableInstance.getDefault()
        .getStringTopic("/Metadata/SHA")
        .publish()
        .set(BuildConstants.GIT_SHA);
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    Threads.setCurrentThreadPriority(true, 99);

    double startTime = HALUtil.getFPGATime();

    CommandScheduler.getInstance().run();

    DogLog.log("Loop Time/Command Scheduler", (HALUtil.getFPGATime() - startTime) / 1000);

    double endTime = HALUtil.getFPGATime();

    m_robotContainer.periodic();

    DogLog.log("Loop Time/Robot Container", (HALUtil.getFPGATime() - endTime) / 1000);

    gcStatsCollector.update();

    double currentTime = HALUtil.getFPGATime();
    DogLog.log("Loop Time/Total", (currentTime - prevTime) / 1000);
    prevTime = currentTime;

    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private static final class GcStatsCollector {
    private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
    private final long[] lastTimes = new long[gcBeans.size()];
    private final long[] lastCounts = new long[gcBeans.size()];

    public void update() {
      long accumTime = 0;
      long accumCounts = 0;
      for (int i = 0; i < gcBeans.size(); i++) {
        long gcTime = gcBeans.get(i).getCollectionTime();
        long gcCount = gcBeans.get(i).getCollectionCount();
        accumTime += gcTime - lastTimes[i];
        accumCounts += gcCount - lastCounts[i];

        lastTimes[i] = gcTime;
        lastCounts[i] = gcCount;
      }

      DogLog.log("GC/GCTimeMS", (double) accumTime);
      DogLog.log("GC/GCCounts", (double) accumCounts);
    }
  }
}
