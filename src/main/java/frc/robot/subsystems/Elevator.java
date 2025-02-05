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
import edu.wpi.first.wpilibj.SensorUtil;
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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.WaysItCanMove;

public final class Elevator extends SubsystemBase implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox =
      DCMotor.getNEO(ElevatorConstants.Motor.kHowManyInGearbox);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      // TODO also move these magic numbers to constants
      new ProfiledPIDController(
          10,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxVelocity));

  // TODO move these magic number s to constants
  ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0, 0.762, 0.762, 0);

  // the encoderssssss
  private final Encoder m_encoder =
      new Encoder(ElevatorConstants.Encoder.kAChannel, ElevatorConstants.Encoder.kBChannel);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // the motorsssssss
  private final SparkMax m_motor =
      new SparkMax(ElevatorConstants.Motor.kPort, MotorType.kBrushless);
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
          ElevatorConstants.Measurements.kGearing,
          ElevatorConstants.Measurements.kWeight,
          ElevatorConstants.Measurements.kDrumRadius,
          ElevatorConstants.Measurements.kBottomHeight,
          ElevatorConstants.Measurements.kTopHeight,
          true,
          ElevatorConstants.Measurements.kBottomHeight,
          ElevatorConstants.Measurements.kStandardDeviation);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d =
      new Mechanism2d(ElevatorConstants.Mechanism2d.kWidth, ElevatorConstants.Mechanism2d.kHeight);
  private final MechanismRoot2d m_mech2dRoot =
      m_mech2d.getRoot(
          "Elevator Root",
          ElevatorConstants.Mechanism2d.kXDistance,
          ElevatorConstants.Mechanism2d.kYDistance);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  //TODO add a sensor to change this when it camt move down
  //somethin like.... SENSOR... event.. its really short... change one of these to false...
  private boolean canMoveDown = true;
  private boolean canMoveUp = true;
  //nowhere, up, or down
  private WaysItCanMove wheresItGoin = ElevatorConstants.WaysItCanMove.nowhere;

  /** Subsystem constructor. */
  public Elevator() {
    m_encoder.setDistancePerPulse(ElevatorConstants.Encoder.kDistancePerPulse);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    // TODO the old one used set m_motorSim.getSpeed() but sparkmax doesnt have that soo its broke now
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(ElevatorConstants.kUpdateFrequency);

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
    //im gettin a loop overrun on this a lot. IS the computer stupid? Its like 5 logic gates and 4 memory lookups
    if (
      (wheresItGoin == ElevatorConstants.WaysItCanMove.up && !canMoveUp) ||
      (wheresItGoin == ElevatorConstants.WaysItCanMove.down && !canMoveDown) || 
      m_elevatorSim.hasHitUpperLimit() || m_elevatorSim.hasHitLowerLimit()) {
      // i think the indentation is wrong but we aint got prettier so nobody ll know,...
        stop();
    }
  }

  /**
   * Run control loop to reach and maintain goal.
   * @param goal the position to maintain
   * @param speed how fast to go 0 (slow) to 1 (fast)
   */
  public void reachGoal(double goal, double speed) {
    m_controller.setGoal(goal);
    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_encoder.getDistance());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput + feedforwardOutput);
    //im just adding random shit at this point to get it to work 
    m_elevatorSim.setInputVoltage(pidOutput + feedforwardOutput);
    m_motorSim.setBusVoltage(feedforwardOutput);
    //teto
    SmartDashboard.putNumber("Position meters", m_encoderSim.getDistance());
    SmartDashboard.putString("Movement", wheresItGoin.toString());
  }

  /**
   * Run control loop to reach and maintain goal.
   * @param goal where to go, IDK the units!!
   * @return nuthin!
   */
  public void reachGoal(double goal) {
    reachGoal(goal, 1);
    //i hope i dont get stack overflow! !! !! I>mgiht though im stupid
  }

  /** makes it go up until stop() */
  public Command up() {
    return this.run(
        () -> {
          if (canMoveUp) {
            wheresItGoin = ElevatorConstants.WaysItCanMove.up;
            reachGoal(ElevatorConstants.Measurements.kTopHeight);
          }
        });
  }

  /** makes it go towards the bottom until stop() */
  public Command down() {
    return this.run(
        () -> {
          if (canMoveDown) {
            wheresItGoin = ElevatorConstants.WaysItCanMove.down;
            reachGoal(ElevatorConstants.Measurements.kBottomHeight);
          }
        });
  }

  /** Stop the control loop and motor output. */
  public Command stop() {
    return this.run(
        () -> {
          wheresItGoin = ElevatorConstants.WaysItCanMove.nowhere;
          SmartDashboard.putString("Movement", wheresItGoin.toString());
          m_controller.setGoal(0.0);
          m_motor.set(0.0);
          m_motor.stopMotor();
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
