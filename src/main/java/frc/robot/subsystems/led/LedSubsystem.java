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
  }

  private final SolidColor[] white =
      new SolidColor[] {
        new SolidColor(0, LedConstants.IndexMax).withColor(new RGBWColor(255, 255, 255))
      };

  private final SolidColor[] green =
      new SolidColor[] {
        new SolidColor(0, LedConstants.IndexMax).withColor(new RGBWColor(0, 255, 0))
      };

  private final SolidColor[] red =
      new SolidColor[] {
        new SolidColor(0, LedConstants.IndexMax).withColor(new RGBWColor(255, 0, 0))
      };

  private final SolidColor[] disable =
      new SolidColor[] {new SolidColor(0, LedConstants.IndexMax).withColor(new RGBWColor(0, 0, 0))};

  public Command disable() {
    return run(
        () -> {
          for (var solidColor : disable) {
            candle.setControl(solidColor);
          }
        });
  }

  public Command solidGreen() {
    return run(
        () -> {
          for (var solidColor : green) {
            candle.setControl(solidColor);
          }
        });
  }

  public Command solidRed() {
    return run(
        () -> {
          for (var solidColor : red) {
            candle.setControl(solidColor);
          }
        });
  }

  public Command countDown() {
    return Commands.sequence(
        solidGreen(),
        Commands.waitSeconds(1),
        setCountDown(5),
        Commands.waitSeconds(1),
        setCountDown(4),
        Commands.waitSeconds(1),
        setCountDown(3),
        Commands.waitSeconds(1),
        setCountDown(2),
        Commands.waitSeconds(1),
        setCountDown(1),
        Commands.waitSeconds(1),
        
        solidRed());
  }

  public Command setCountDown(int count) {
    return runOnce(
        () -> {
          SolidColor[] countdown =
              new SolidColor[] {
                new SolidColor(LedConstants.IndexMax * count / 5, LedConstants.IndexMax)
                    .withColor(new RGBWColor(0, 0, 0))
              };
          for (var solidColor : countdown) {
            candle.setControl(solidColor);
          }
        });
  }
}
