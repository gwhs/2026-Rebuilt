package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  final CANdle candle = new CANdle(LedConstants.CANdleID, LedConstants.CANdleCANBus);

  public LedSubsystem() {
    CANdleConfiguration conf = new CANdleConfiguration();
    conf.LED.StripType = StripTypeValue.RGB;
    conf.LED.BrightnessScalar = 1.0;
    conf.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
  }

  public void colorSetter(Color color) {
    this.candle.setControl(
        new SolidColor(0, LedConstants.IndexMax).withColor(new RGBWColor(color)));
  }

  public Command setColor(Color color) {
    return this.runOnce(
        () -> {
          colorSetter(color);
        });
  }
}
