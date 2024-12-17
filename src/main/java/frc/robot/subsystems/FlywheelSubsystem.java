// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

@Logged
public class FlywheelSubsystem extends SubsystemBase {
  private static final int kFlywheelCanId = 21;
  // 0.00032 kg * m^2 is for 1 4"x1.5" Colson
  private static final double kColsonMomentOfInertia = 2 * 0.00032; // kg * m^2

  private final SparkMax m_sparkMax = new SparkMax(kFlywheelCanId, MotorType.kBrushless);
  private final SparkMaxConfig m_config = new SparkMaxConfig();
  private final RelativeEncoder m_encoder = m_sparkMax.getEncoder();
  private final SparkClosedLoopController m_controller = m_sparkMax.getClosedLoopController();

  private final DCMotor m_gearbox = DCMotor.getNEO(1);
  private final SparkMaxSim m_sparkMaxSim = new SparkMaxSim(m_sparkMax, m_gearbox);
  
  private final FlywheelSim m_flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(m_gearbox, kColsonMomentOfInertia, 1), m_gearbox);

  public FlywheelSubsystem() {
    m_config.idleMode(IdleMode.kCoast).smartCurrentLimit(50, 50);
    m_config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0001, 0, 0.0)
        // .velocityFF(1 / 5600)  // doesn't appear to be working in sim
        .outputRange(-1, 1);

    m_sparkMax.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    stop();
  }

  @Override
  public void simulationPeriodic() {
    double timestep = 20e-3;
    m_flywheelSim.setInputVoltage(getAppliedVoltage());
    m_flywheelSim.update(timestep);
    m_sparkMaxSim.iterate(m_flywheelSim.getAngularVelocityRPM(), getInputVoltage(), timestep);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(this.getOutputCurrent()));
  }

  public double getRPM() {
    return m_encoder.getVelocity();
  }

  public void setDesiredRPM(double rpm) {
    // m_controller.setReference(rpm, ControlType.kVelocity);
    m_controller.setReference(rpm, ControlType.kVelocity, 0, 12 * rpm / 5676);
  }

  public void stop() {
    m_sparkMax.stopMotor();
  }

  public Command setRPMCommand(double rpm) {
    return run(() -> setDesiredRPM(rpm));
  }

  public Command setRPMCommand(DoubleSupplier rpm) {
    return run(() -> setDesiredRPM(rpm.getAsDouble()));
  }

  public Command stopCommand() {
    return run(this::stop);
  }

  public double getAppliedOutput() {
    return m_sparkMax.getAppliedOutput();
  }

  public double getInputVoltage() {
    return RobotController.getInputVoltage();
  }

  public double getAppliedVoltage() {
    return m_sparkMax.getAppliedOutput() * getInputVoltage();
  }

  public double getOutputCurrent() {
    return m_sparkMax.getOutputCurrent();
  }
}
