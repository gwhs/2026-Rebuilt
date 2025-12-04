package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RobotVisualizer {
  private final Mechanism2d panel = new Mechanism2d(ROBOT_LENGTH, ROBOT_LENGTH * 5);

  // Robot Constants
  public static final double ROBOT_LENGTH = Units.inchesToMeters(27.5);

  public Color colorRed = Color.kFirstRed;

  // Code for the stick figure of each subsystems
  MechanismRoot2d root = panel.getRoot("elevator", ROBOT_LENGTH / 2, 0.078);
  MechanismLigament2d m_elevator =
      root.append(new MechanismLigament2d("elevatorL", 1.5, 90, 10, new Color8Bit(colorRed)));
  // arm
  MechanismLigament2d m_arm =
      m_elevator.append(
          new MechanismLigament2d(
              "arm", Units.inchesToMeters(20), 90, 10, new Color8Bit(Color.kWhite)));

  // ground intake
  MechanismRoot2d root2 = panel.getRoot("ground", (ROBOT_LENGTH / 2) - 0.3, 0.078);
  MechanismLigament2d m_ground_intake =
      root2.append(
          new MechanismLigament2d(
              "ground intake",
              Units.inchesToMeters(18.75),
              90,
              10,
              new Color8Bit(Color.kAliceBlue)));

  public RobotVisualizer() {}

  public void update() {}
}
