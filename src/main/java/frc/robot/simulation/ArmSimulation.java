// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmSimulation implements AutoCloseable {
  public static final double kArmReduction = 200;
  public static final double kArmMass = 8.0; // Kilograms
  public static final double kArmLength = Units.inchesToMeters(30);
  public static final double kMinAngleRads = Units.degreesToRadians(-75);
  public static final double kMaxAngleRads = Units.degreesToRadians(255);

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);
  private PWMSparkMax m_motor;

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private SingleJointedArmSim m_armSim;
  private EncoderSim m_encoderSim;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d;
  private final MechanismRoot2d m_armPivot;
  private final MechanismLigament2d m_armTower;
  private final MechanismLigament2d m_arm;

  /** Subsystem constructor. */
  public ArmSimulation(PWMSparkMax motor, Encoder encoder) {
    m_encoderSim = new EncoderSim(encoder);
    m_motor = motor;

    m_armSim =
    new SingleJointedArmSim(
        m_armGearbox,
        kArmReduction,
        SingleJointedArmSim.estimateMOI(kArmLength, kArmMass),
        kArmLength,
        kMinAngleRads,
        kMaxAngleRads,
        true,
        new MatBuilder<>(Nat.N1(),Nat.N1()).fill(encoder.getDistancePerPulse()) // Add noise with a std-dev of 1 tick
        );

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  m_mech2d = new Mechanism2d(60, 60);
  m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));
  }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }

  @Override
  public void close() {
    m_mech2d.close();
    m_armPivot.close();
    m_arm.close();
  }
}
