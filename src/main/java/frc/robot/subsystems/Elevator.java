/**
 * @fileoverview the elevator, it has 1 motor
 */
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Also credit to this repo here most of it is copied n pasted from this
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java

package frc.robot.subsystems;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameConstants;

@Logged
public final class Elevator extends SubsystemBase implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox =
      DCMotor.getNEO(ElevatorConstants.Motor.kHowManyInGearbox);

  /**
   * Elevator motor module uses a conversion factor of {@link
   * ElevatorConstants.Measurements#kDrumRadius}. PID/encoder use meters instead of rotations.
   */
  private final SparkMax m_motor =
      new SparkMax(ElevatorConstants.Motor.kCanIdMotor1, MotorType.kBrushless);

  private final SparkMax m_motor2 =
      new SparkMax(ElevatorConstants.Motor.kCanIdMotor2, MotorType.kBrushless);
  private final SparkAbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder();
  private final SparkClosedLoopController m_closedLoopController =
      m_motor.getClosedLoopController();

  private final ElevatorFeedforward m_feedForward =
      new ElevatorFeedforward(
          ElevatorConstants.FeedForward.Ks,
          ElevatorConstants.FeedForward.Kg,
          ElevatorConstants.FeedForward.Kv,
          ElevatorConstants.FeedForward.Ka);

  // Sim classes
  private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);
  private final SparkAbsoluteEncoderSim m_encoderSim = new SparkAbsoluteEncoderSim(m_motor);

  private final PIDController m_pid = new PIDController(10, 2.5, 0);

  // Simulation classes help us simulate what's going on, including gravity.
  // Tentative values for all from CAD. drumRadiusMeters/minmax height is off.
  private final ElevatorSim m_elevatorSim =
      // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/ElevatorSim.html
      new ElevatorSim(
          m_elevatorGearbox,
          ElevatorConstants.Measurements.kGearing,
          ElevatorConstants.kWeight,
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

  // nowhere, up, or down
  private ElevatorConstants.WaysItCanMove wheresItGoin = ElevatorConstants.WaysItCanMove.nowhere;
  // stores what reachGoal() was called with last time
  private double lastGoal = 0;

  /** Subsystem constructor. */
  public Elevator() {
    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);

    m_motor.configure(
        Configs.ElevatorConfigs.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // This is a variable so it doesn't have to be called too often
    double currentPosition = m_elevatorSim.getPositionMeters();
    // once its past the "slow down distance", how close it is to the goal
    // 0.1 is very close, 0 means its right at that distance, bigger than
    // 0.1 means it's not time to slow down yet
    double percentUntilStop =
        Math.abs(currentPosition - lastGoal) / ElevatorConstants.Measurements.kSlowDownDistance;
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    if (Math.random() < 0.001) {
      reachGoal(Math.random() * ElevatorConstants.Measurements.kTopHeight);
    }

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(ElevatorConstants.kUpdateFrequency);

    // Required to keep a SparkMax working
    m_motorSim.iterate(
        m_elevatorSim.getVelocityMetersPerSecond(),
        RobotController.getBatteryVoltage(),
        ElevatorConstants.kUpdateFrequency);

    SmartDashboard.putNumber("percent till stop distance", percentUntilStop);

    if (lastGoal > currentPosition) {
      wheresItGoin = ElevatorConstants.WaysItCanMove.up;
    } else if (lastGoal < currentPosition) {
      wheresItGoin = ElevatorConstants.WaysItCanMove.down;
    }

    SmartDashboard.putNumber("Motor output", m_motorSim.getAppliedOutput());
    SmartDashboard.putNumber("Position meters", currentPosition);
    SmartDashboard.putNumber("Goal", lastGoal);
    SmartDashboard.putString("Movement", wheresItGoin.toString());

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setPosition(currentPosition);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    updateTelemetry();
  }

  /**
   * Goes to a place in meters
   *
   * @param goal where 2 go
   * @throws Error if the goal is bigger than max height plus arm height or less than 0
   */
  public void reachGoal(double goal) {
    // Error handling
    // if (goal > ElevatorConstants.Measurements.kTopHeight + ArmConstants.kHeight
    //     || goal < ElevatorConstants.Measurements.kBottomHeight) {
    //   throw new Error(
    //       "Goal is out of range! Should be between"
    //           + ElevatorConstants.Measurements.kTopHeight
    //           + ArmConstants.kHeight
    //           + "and "
    //           + ElevatorConstants.Measurements.kBottomHeight
    //           + " meters. Attempted goal: "
    //           + goal);
    // }
    // record of where it was goin last time
    lastGoal = goal;
    m_closedLoopController.setReference(
        m_pid.calculate(m_elevatorSim.getPositionMeters(), goal), ControlType.kPosition);
  }

  /**
   * Stop the control loop and motor output. WARNING if you called a method before it will stop in the
   * middle.
   */
  public Command stop() {
    return this.runOnce(
        () -> {
          wheresItGoin = ElevatorConstants.WaysItCanMove.nowhere;
          SmartDashboard.putString("Movement", wheresItGoin.toString());
          m_closedLoopController.setReference(0, ControlType.kMAXMotionPositionControl);
          m_motor.set(0.0);
          m_motor.stopMotor();
        });
  }

  /** makes it stop but a void and no command TODO better name */
  public void stopButNotCommand() {
    wheresItGoin = ElevatorConstants.WaysItCanMove.nowhere;
    SmartDashboard.putString("Movement", wheresItGoin.toString());
    m_closedLoopController.setReference(0, ControlType.kMAXMotionPositionControl);
    m_motor.set(0.0);
    m_motor.stopMotor();
  }

  /**
   * GO to branch
   *
   * @param level which branch
   */
  public Command toBranch(GameConstants.ReefLevels level) {
    return this.runOnce(
        () -> {
          switch (level) {
            case l1:
              {
                reachGoal(GameConstants.Positions.kReefL1);
                break;
              }
            case l2:
              {
                reachGoal(GameConstants.Positions.kReefL2);
                break;
              }
            case l3:
              {
                reachGoal(GameConstants.Positions.kReefL3);
                break;
              }
            case l4:
              {
                reachGoal(GameConstants.Positions.kReefL4);
                break;
              }
          }
        });
  }

  /** go to the bottomish */
  public Command toFloorIntake() {
    return this.runOnce(() -> reachGoal(GameConstants.Positions.kFloorIntake));
  }

  /** go to human coral putting station */
  public Command toFeeder() {
    return this.runOnce(() -> reachGoal(GameConstants.Positions.kFeeder));
  }

  public Command toStored() {
    return this.runOnce(() -> reachGoal(GameConstants.Positions.kCoralStorage));
  }

  /** go to elevator upper limit */
  public Command toTop() {
    return this.runOnce(() -> reachGoal(ElevatorConstants.Measurements.kTopHeight));
  }

  /** go to elevator lower limit */
  public Command toBottom() {
    return this.runOnce(() -> reachGoal(ElevatorConstants.Measurements.kBottomHeight));
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(m_encoder.getPosition());
  }

  @Override
  public void close() {
    m_motor.close();
    m_mech2d.close();
  }
}
