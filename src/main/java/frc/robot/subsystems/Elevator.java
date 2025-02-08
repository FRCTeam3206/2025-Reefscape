/**
 * @fileoverview the elevator, it has 1 motor
 */
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Also credit to this repo here most of it is copied n pasted from this
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public final class Elevator extends SubsystemBase implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox =
      DCMotor.getNEO(ElevatorConstants.Motor.kHowManyInGearbox);

  private final SparkMax m_motor = new SparkMax(ElevatorConstants.Motor.kPort, MotorType.kBrushless);
  private final AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder();
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();

  private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);

  // Changeable setpoint for current elevator state
  public double elevatorSetpoint = 0;

  // Simulation classes help us simulate what's going on, including gravity.
  // Tentative values for all from CAD. drumRadiusMeters/minmax height is off.
  private final ElevatorSim m_elevatorSim =
  /*they should really puy the link to the docs somewhere in the class definition
   * IDK how javadoc works but jsdoc has @see something like THHat
   * ANWYAYS heres the link lel
   *  https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/ElevatorSim.html
   */
      new ElevatorSim(
          m_elevatorGearbox,
          ElevatorConstants.Measurements.kGearing,
          ElevatorConstants.Measurements.kWeight,
          ElevatorConstants.Measurements.kDrumRadius,
          ElevatorConstants.Measurements.kBottomHeight,
          ElevatorConstants.Measurements.kTopHeight,
          true,
          ElevatorConstants.Measurements.kBottomHeight,
          ElevatorConstants.Measurements.kStandardDeviation);

// TODO move this to overarching class
// Create a Mechanism2d visualization of the elevator
//   private final Mechanism2d m_mech2d =
//       new Mechanism2d(ElevatorConstants.Mechanism2d.kWidth, ElevatorConstants.Mechanism2d.kHeight);
//   private final MechanismRoot2d m_mech2dRoot =
//       m_mech2d.getRoot(
//           "Elevator Root",
//           ElevatorConstants.Mechanism2d.kXDistance,
//           ElevatorConstants.Mechanism2d.kYDistance);
//   private final MechanismLigament2d m_elevatorMech2d =
//       m_mech2dRoot.append(
//           new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  /** Subsystem constructor. */
  public Elevator() {
  }

  // TODO: could be a strange way to change the elevator's setpoint, refactor if necessary
  // TODO: implement limit switch functionality
  private void moveToSetpoint() {
    m_controller.setReference(elevatorSetpoint, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void periodic() {
    moveToSetpoint();
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(ElevatorConstants.kUpdateFrequency);

    // Required to keep a SparkMax working
    m_motorSim.iterate(
      m_elevatorSim.getVelocityMetersPerSecond(), 
      RoboRioSim.getVInVoltage(), 
      ElevatorConstants.kUpdateFrequency
    );
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  public double getCurrentDrawAmps() {
    return m_elevatorSim.getCurrentDrawAmps();
  }

  public double getElevatorHeight() {
    return m_encoder.getPosition() * ElevatorConstants.Measurements.kDrumRadius;
  }

  @Override
  public void close() {
    m_motor.close();
  }
}