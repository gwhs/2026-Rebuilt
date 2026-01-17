package frc.robot.subsystems.groundIntakePivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class GroundIntakePivotIOReal implements GroundIntakePivotIO {
  private TalonFX groundIntakePivotMotor = new TalonFX(GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_MOTOR_ID, "rio");
  private CANcoder groundIntakePivotEncoder = new CANcoder(GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_ENCODER_ID, "rio");
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final StatusSignal<Double> groundIntakePivotPIDGoal = GroundIntakePivotSubsystem.getClosedLoopReference();
  private final StatusSignal<Voltage> groundIntakePivotMotorVoltage = groundIntakePivotMotor.getMotorVoltage();
  private final StatusSignal<Voltage> groundIntakePivotSupplyVoltage = groundIntakePivotMotor.getSupplyVoltage();
  private final StatusSignal<Temperature> groundIntakePivotDeviceTemp = groundIntakePivotMotor.getDeviceTemp();
  private final StatusSignal<Current> groundIntakePivotStatorCurrent = groundIntakePivotMotor.getStatorCurrent();
  private final StatusSignal<Angle> groundIntakePivotEncoderPosition = groundIntakePivotEncoder.getPosition();
  private final StatusSignal<Angle> groundIntakePivotPosition = groundIntakePivotMotor.getPosition();
  private final StatusSignal<Double> groundIntakePivotAngleError = groundIntakePivotMotor.getClosedLoopError();
  private final StatusSignal<Double> feedForwardOutput = groundIntakePivotMotor.getClosedLoopFeedForward();
  private final StatusSignal<Double> closedLoopOutput = groundIntakePivotMotor.getClosedLoopOutput();
  private final StatusSignal<Double> closedLoopProportionalOutput =
      groundIntakePivotMotor.getClosedLoopProportionalOutput();
  private final StatusSignal<Double> closedLoopIntegral = groundIntakePivotMotor.getClosedLoopIntegratedOutput();
  private final StatusSignal<Double> closedLoopDerivative =
      groundIntakePivotMotor.getClosedLoopDerivativeOutput();

  private final Alert groundIntakePivotMotorConnectedAlert =
      new Alert("Ground Intake Pivot motor not connected", AlertType.kError);

  private final Alert groundIntakePivotEncoderConnectedAlert =
      new Alert("Ground Intake Pivot CANcoder not connected", AlertType.kError);

  private boolean m_emergencyMode;

  public GroundIntakePivotIOReal() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorOutput = talonFXConfigs.MotorOutput;
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    SoftwareLimitSwitchConfigs softwareLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
    CurrentLimitsConfigs currentConfig = talonFXConfigs.CurrentLimits;
    FeedbackConfigs feedbackConfigs = talonFXConfigs.Feedback;
    m_request.EnableFOC = true; // add FOC
    slot0Configs.kS = 0.3; // Add 0.25 V output to overcome static friction
    slot0Configs.kG = 0.5; // Add 0 V to overcome gravity
    slot0Configs.kV = 6; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.2; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 43; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 10; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.withGravityType(GravityTypeValue.Cosine);

    feedbackConfigs.FeedbackRotorOffset = 0;
    feedbackConfigs.FeedbackRemoteSensorID = GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_ENCODER_ID;
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    feedbackConfigs.RotorToSensorRatio = GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_GEAR_RATIO;
    feedbackConfigs.SensorToMechanismRatio = 1;

    motionMagicConfigs.MotionMagicCruiseVelocity = GroundIntakePivotConstants.MAX_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = GroundIntakePivotConstants.MAX_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = 0; // Target jerk of 1600 rps/s/s (0.1 seconds)

    motorOutput.NeutralMode = NeutralModeValue.Brake;
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    softwareLimitSwitch.ForwardSoftLimitEnable = true;
    softwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_UPPER_BOUND);
    softwareLimitSwitch.ReverseSoftLimitEnable = true;
    softwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_LOWER_BOUND);

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(30);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = groundIntakePivotMotor.getConfigurator().apply(talonFXConfigs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        groundIntakePivotPIDGoal,
        groundIntakePivotStatorCurrent,
        feedForwardOutput,
        closedLoopOutput,
        closedLoopProportionalOutput,
        closedLoopIntegral,
        closedLoopDerivative);

    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cc_cfg.MagnetSensor.withMagnetOffset(
        Units.degreesToRotations(GroundIntakePivotConstants.MAGNET_OFFSET_DEGREES));
    for (int i = 0; i < 5; i++) {
      status = groundIntakePivotEncoder.getConfigurator().apply(cc_cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  public double getPIDGoalDegrees() {
    return Units.rotationsToDegrees(groundIntakePivotPIDGoal.getValueAsDouble());
  }

  // set groundIntakePivot angle in degrees
  @Override
  public void setAngle(double angle) {
    groundIntakePivotMotor.setControl(m_request.withPosition(Units.degreesToRotations(angle)));
  }

  @Override
  public double getPosition() {
    return Units.rotationsToDegrees(groundIntakePivotEncoderPosition.getValueAsDouble());
  }

  public double getPositionError() {
    return groundIntakePivotAngleError.getValueAsDouble();
  }

  /**
   * @param volts the voltage to set to
   */
  public void setVoltage(double volts) {
    if (m_emergencyMode == true) {
      groundIntakePivotMotor.setControl(m_voltReq.withOutput(0));
    } else {
      groundIntakePivotMotor.setControl(m_voltReq.withOutput(volts));
    }
  }

  public void setEmergencyMode(boolean emergency) {
    m_emergencyMode = emergency;
    setVoltage(0);
  }

  @Override
  public void update() {

    boolean groundIntakePivotConnected =
        (BaseStatusSignal.refreshAll(
                groundIntakePivotPIDGoal,
                groundIntakePivotMotorVoltage,
                groundIntakePivotSupplyVoltage,
                groundIntakePivotDeviceTemp,
                groundIntakePivotStatorCurrent,
                groundIntakePivotPosition,
                groundIntakePivotEncoderPosition,
                groundIntakePivotAngleError,
                feedForwardOutput,
                closedLoopOutput,
                closedLoopProportionalOutput,
                closedLoopIntegral,
                closedLoopDerivative)
            .isOK());
    DogLog.log("GroundIntakePivot/Motor/pid goal", Units.rotationsToDegrees(groundIntakePivotPIDGoal.getValueAsDouble()));
    DogLog.log("GroundIntakePivot/Motor/motor voltage", groundIntakePivotMotorVoltage.getValueAsDouble());
    DogLog.log("GroundIntakePivot/Motor/supply voltage", groundIntakePivotSupplyVoltage.getValueAsDouble());
    DogLog.log("GroundIntakePivot/Motor/device temp", groundIntakePivotDeviceTemp.getValueAsDouble());
    DogLog.log("GroundIntakePivot/Motor/stator current", groundIntakePivotStatorCurrent.getValueAsDouble());
    DogLog.log("GroundIntakePivot/Motor/Connected", groundIntakePivotConnected);
    DogLog.log("GroundIntakePivot/Encoder/encoder position", getPosition());
    DogLog.log("GroundIntakePivot/Encoder/Connected", groundIntakePivotEncoder.isConnected());
    DogLog.log("GroundIntakePivot/Motor/feed forward", feedForwardOutput.getValueAsDouble());
    DogLog.log("GroundIntakePivot/Motor/closed loop output", closedLoopOutput.getValueAsDouble());
    DogLog.log(
        "GroundIntakePivot/Motor/closed loop proportional", closedLoopProportionalOutput.getValueAsDouble());
    DogLog.log("GroundIntakePivot/Motor/closed loop integral", closedLoopIntegral.getValueAsDouble());
    DogLog.log("GroundIntakePivot/Motor/closed loop derivative", closedLoopDerivative.getValueAsDouble());

    groundIntakePivotMotorConnectedAlert.set(!groundIntakePivotConnected);
    groundIntakePivotEncoderConnectedAlert.set(!groundIntakePivotEncoder.isConnected());

    if (m_emergencyMode == true) {
      setVoltage(0);
    }
  }
}

