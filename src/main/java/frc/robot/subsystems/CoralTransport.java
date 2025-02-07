/**
 * @fileoverview the elevator, it has 1 motor
 */
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Also credit to this repo here most of it is copied n pasted from this
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralTransportConstants;
import frc.robot.Constants.CoralTransportConstants.WaysItCanMove;
import frc.robot.Robot;

public final class CoralTransport extends SubsystemBase implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox =
      DCMotor.getNEO(CoralTransportConstants.Motor.kHowManyInGearbox);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          Constants.CoralTransportConstants.Controller.Kp,
          Constants.CoralTransportConstants.Controller.Ki,
          Constants.CoralTransportConstants.Controller.Kd,
          new TrapezoidProfile.Constraints(
              CoralTransportConstants.kMaxVelocity, CoralTransportConstants.kMaxAcceleration));

  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.CoralTransportConstants.FeedForward.Ks,
          Constants.CoralTransportConstants.FeedForward.Kg,
          Constants.CoralTransportConstants.FeedForward.Kv,
          Constants.CoralTransportConstants.FeedForward.Ka);

  // the encoderssssss
  private final Encoder m_encoder =
      new Encoder(
          CoralTransportConstants.Encoder.kAChannel, CoralTransportConstants.Encoder.kBChannel);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  private final SparkMax m_motor =
      new SparkMax(CoralTransportConstants.Motor.kPort, MotorType.kBrushless);
  private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);

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
          CoralTransportConstants.Measurements.kGearing,
          CoralTransportConstants.Measurements.kWeight,
          CoralTransportConstants.Measurements.kDrumRadius,
          CoralTransportConstants.Measurements.kBottomHeight,
          CoralTransportConstants.Measurements.kTopHeight,
          true,
          CoralTransportConstants.Measurements.kBottomHeight,
          CoralTransportConstants.Measurements.kStandardDeviation);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d =
      new Mechanism2d(
          CoralTransportConstants.Mechanism2d.kWidth, CoralTransportConstants.Mechanism2d.kHeight);
  private final MechanismRoot2d m_mech2dRoot =
      m_mech2d.getRoot(
          "Elevator Root",
          CoralTransportConstants.Mechanism2d.kXDistance,
          CoralTransportConstants.Mechanism2d.kYDistance);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  // TODO add a sensor to change this when it camt move down
  // somethin like.... SENSOR... event.. its really short... change one of these to false...
  private boolean canMoveDown = true;
  private boolean canMoveUp = true;
  // nowhere, up, or down
  private WaysItCanMove wheresItGoin = CoralTransportConstants.WaysItCanMove.nowhere;

  /** Subsystem constructor. */
  public CoralTransport() {
    m_encoder.setDistancePerPulse(CoralTransportConstants.Encoder.kDistancePerPulse);

    if (Robot.isSimulation()) {
      // Publish Mechanism2d to SmartDashboard
      // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
      SmartDashboard.putData("Elevator Sim", m_mech2d);
    }
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    // TODO the old one used set m_motorSim.getSpeed() but sparkmax doesnt have that soo its broke
    // now
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(CoralTransportConstants.kUpdateFrequency);

    // Required to keep a SparkMax working
    m_motorSim.iterate(
        m_elevatorSim.getVelocityMetersPerSecond(),
        RoboRioSim.getVInVoltage(),
        CoralTransportConstants.kUpdateFrequency);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    updateTelemetry();
  }

  /*
   * stops if it cant move up or it cant move down
   * So we dont break things!!!
   */
  public void periodic() {
    /*im gettin a loop overrun on this a lot. IS the computer stupid?
    Its like 5 logic gates and 4 memory lookups*/
    /*
    if (
      (wheresItGoin == CoralTransportConstants.WaysItCanMove.up && !canMoveUp) ||
      (wheresItGoin == CoralTransportConstants.WaysItCanMove.down && !canMoveDown) ||
      m_elevatorSim.hasHitUpperLimit() || m_elevatorSim.hasHitLowerLimit()) {
      // i think the indentation is wrong but we aint got prettier so nobody ll know,...
        changeSpeed(0);
    }
    */
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal) {
    m_controller.setGoal(goal);
    double pidOutput = m_controller.calculate(m_encoder.getDistance());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput + feedforwardOutput * CoralTransportConstants.Motor.kSpeed);
    SmartDashboard.putNumber("Position meters", m_encoderSim.getDistance());
    // changes it to Movement: down, Movement: nowhere, or Movement: up
    SmartDashboard.putString("Movement", wheresItGoin.toString());
    // teto
  }

  /**
   * finds the goal to go based on the stick Y
   *
   * @param where 0 to 5 for places it should go, 0 is the bottom level and the rest are first
   *     level, second level, until fourth
   * @return command
   */
  public Command goToPlace(int place) {
    return this.run(
        () -> {
          if (place == 0) {
            reachGoal(CoralTransportConstants.Positions.kBottomLevel);
          } else {
            reachGoal(CoralTransportConstants.Positions.kLevels[place - 1]);
          }
          SmartDashboard.putNumber("goal", place);
        });
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(m_encoder.getDistance());
  }

  @Override
  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech2d.close();
  }
}
