// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.simulation.ArmSimulation;

public class Arm implements AutoCloseable {
  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;

  double m_armSetpoint = 1.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(kMotorPort);

  private ArmSimulation m_ArmSimulation;

  /** Subsystem constructor. */
  public Arm() {
    m_encoder.setDistancePerPulse(kArmEncoderDistPerPulse);

    m_ArmSimulation= new ArmSimulation(m_motor, m_encoder);

    // Set the Arm position setpoint and P constant to Preferences if the keys don't already exist
    Preferences.initDouble("ArmPosition", m_armSetpoint);
  }

  /** Load setpoint and kP from preferences. */
  public void loadPreferences() {
    // Read Preferences for Arm setpoint and kP on entering Teleop
    m_armSetpoint = Preferences.getDouble("ArmPosition", m_armSetpoint);
  }

  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public void reachSetpoint() {
    m_motor.setVoltage(m_armSetpoint);
  }

  public void sendTelemetry() {
    SmartDashboard.putNumber("ArmPosition",Units.radiansToDegrees(m_encoder.getDistance()));
  }

  public void stop() {
    m_motor.set(0.0);
  }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // Call the arm simulation with the motor state
    m_ArmSimulation.simulationPeriodic();
  }

  @Override
  public void close() {
    m_motor.close();
    m_encoder.close();
    m_ArmSimulation.close();
  }
}
