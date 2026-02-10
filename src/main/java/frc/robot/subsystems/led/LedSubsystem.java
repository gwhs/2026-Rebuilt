package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    SmartDashboard.putData("solid green", solidGreen());
  }

  private final SolidColor[] white =
      new SolidColor[] {
        new SolidColor(0, LedConstants.IndexMax).withColor(new RGBWColor(255, 255, 255))
      };

  private final SolidColor[] green =
      new SolidColor[] {
        new SolidColor(0, LedConstants.IndexMax).withColor(new RGBWColor(0, 255, 0))
      };

  public Command solidGreen() {
    return run(
        () -> {
          for (var solidColor : green) {
            candle.setControl(solidColor);
          }
        });
  }
}
