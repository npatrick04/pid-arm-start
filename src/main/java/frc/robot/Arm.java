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

  double m_armSetpoint = 0.0;
  double m_armCommand = 0.0;

  // PID Controller Gains - Use variables so that we can modify at runtime
  double kp = 1.0;
  double ki = 0.0;
  double kd = 0.0;

  PIDController controller = new PIDController(kp,ki,kd);

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
    Preferences.initDouble("kp", kp);
    Preferences.initDouble("ki", ki);
    Preferences.initDouble("kd", kd);
  }

  /** Load setpoint and kP from preferences. */
  public void loadPreferences() {
    // Read Preferences for Arm setpoint and kP on entering Teleop
    m_armSetpoint = Preferences.getDouble("ArmPosition", m_armSetpoint);
    kp = Preferences.getDouble("kp", kp);
    ki = Preferences.getDouble("ki", ki);
    kd = Preferences.getDouble("kd", kd);
    controller.setPID(kp, ki, kd);
  }

  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public void reachSetpoint() {
    double motorCommand = 0.0;

    motorCommand = controller.calculate(m_encoder.getDistance(), Units.degreesToRadians(m_armSetpoint));
    m_armCommand = m_armSetpoint;
    m_motor.setVoltage(motorCommand);
  }

  public void sendTelemetry() {
    SmartDashboard.putNumber("ArmPosition",Units.radiansToDegrees(m_encoder.getDistance()));
    SmartDashboard.putNumber("ArmCommand",m_armCommand);
  }

  public void stop() {
    m_motor.set(0.0);
    m_armCommand = -75.0;
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
