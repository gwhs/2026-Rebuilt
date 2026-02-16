package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  private final CANdle candle = new CANdle(LedConstants.CANdleID, LedConstants.CANdleCANBus);

  public LedSubsystem() {
    CANdleConfiguration conf = new CANdleConfiguration();
    conf.LED.StripType = StripTypeValue.BRG;
    conf.LED.BrightnessScalar = 1.0;
    conf.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    for (int i = 0; i < 6; i++) {
      candle.getConfigurator().apply(conf);
    }

    SmartDashboard.putData("disable", disable());
    SmartDashboard.putData("solid green", solidGreen());
    SmartDashboard.putData("count down", countDown());
    SmartDashboard.putData(" advance count down", countDown(20, 30));
  }

  private final RGBWColor white = new RGBWColor(255, 255, 255);

  private final RGBWColor green = new RGBWColor(0, 255, 0);

  private final RGBWColor red = new RGBWColor(255, 0, 0);

  private final RGBWColor disable = new RGBWColor(0, 0, 0);

  public Command setColor(int startIndex, int endIndex, RGBWColor color) {
    return runOnce(
        () -> {
          for (var solidColor :
              new SolidColor[] {new SolidColor(startIndex, endIndex).withColor(color)}) {
            candle.setControl(solidColor);
          }
        });
  }

  public Command disable() {
    return runOnce(
        () -> {
          for (var solidColor :
              new SolidColor[] {new SolidColor(0, LedConstants.IndexMax).withColor(disable)}) {
            candle.setControl(solidColor);
          }
        });
  }

  public Command solidGreen() {
    return runOnce(
        () -> {
          for (var solidColor :
              new SolidColor[] {new SolidColor(0, LedConstants.IndexMax).withColor(green)}) {
            candle.setControl(solidColor);
          }
        });
  }

  public Command solidRed() {
    return runOnce(
        () -> {
          for (var solidColor :
              new SolidColor[] {new SolidColor(0, LedConstants.IndexMax).withColor(red)}) {
            candle.setControl(solidColor);
          }
        });
  }

  public Command countDown() {
    return Commands.sequence(
        solidGreen(), // 5
        Commands.waitSeconds(1),
        setColor(28, LedConstants.IndexMax, disable), // 4
        Commands.waitSeconds(1),
        setColor(21, LedConstants.IndexMax, disable), // 3
        Commands.waitSeconds(1),
        setColor(14, LedConstants.IndexMax, disable), // 2
        Commands.waitSeconds(1),
        setColor(7, LedConstants.IndexMax, disable), // 1
        Commands.waitSeconds(1),
        solidRed()); // 0
  }

  public Command countDown(int startIndex, int endIndex) {
    return Commands.sequence(
        setColor(startIndex, endIndex, green),
        Commands.waitSeconds(1),
        setColor(startIndex + (4 * (endIndex - startIndex) / 5), endIndex, disable),
        Commands.waitSeconds(1),
        setColor(startIndex + (3 * (endIndex - startIndex) / 5), endIndex, disable),
        Commands.waitSeconds(1),
        setColor(startIndex + (2 * (endIndex - startIndex) / 5), endIndex, disable),
        Commands.waitSeconds(1),
        setColor(startIndex + ((endIndex - startIndex) / 5), endIndex, disable),
        Commands.waitSeconds(1),
        setColor(startIndex, endIndex, red));
  }
  // 0 - 19, 20 - 30
}
