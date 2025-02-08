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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameConstants;

public final class Elevator extends SubsystemBase implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox =
      DCMotor.getNEO(ElevatorConstants.Motor.kHowManyInGearbox);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
    new ProfiledPIDController(
      Constants.ElevatorConstants.Controller.Kp,
      Constants.ElevatorConstants.Controller.Ki,
      Constants.ElevatorConstants.Controller.Kd,
      new TrapezoidProfile.Constraints(
        ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxVelocity));

  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
    Constants.ElevatorConstants.FeedForward.Ks, 
    Constants.ElevatorConstants.FeedForward.Kg, 
    Constants.ElevatorConstants.FeedForward.Kv, 
    Constants.ElevatorConstants.FeedForward.Ka
  );

  // the encoderssssss
  private final Encoder m_encoder =
    new Encoder(ElevatorConstants.Encoder.kAChannel, ElevatorConstants.Encoder.kBChannel);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);


  private final SparkMax m_motor = new SparkMax(ElevatorConstants.Motor.kPort, MotorType.kBrushless);
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
      //this spacing is so bad. Every time I see it i go blind
      //BUT spotless apply is just gonna put it back to being ugly... smh...
      true,
      ElevatorConstants.Measurements.kBottomHeight,
      ElevatorConstants.Measurements.kStandardDeviation
    );

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

  //nowhere, up, or down
  private ElevatorConstants.WaysItCanMove wheresItGoin = 
    ElevatorConstants.WaysItCanMove.nowhere;
  //stores what reachGoal() was called with last time
  private double lastGoal = 0;

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

    // Required to keep a SparkMax working
    m_motorSim.iterate(
      m_elevatorSim.getVelocityMetersPerSecond(), 
      RobotController.getBatteryVoltage(), 
      ElevatorConstants.kUpdateFrequency
    );


    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    m_elevatorMech2d.setLength(m_encoderSim.getDistance());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
          m_elevatorSim.getCurrentDrawAmps()
    ));

    updateTelemetry();
  }

  /**
   * Goes to a place
   * corrie said the safety is going outside of this so this doenst check for that
   * @param goal the place to go, idk the units
   * @param speed how fast to go 0 (slow) to 1 (fast)
   */
  public void reachGoal(double goal, double speed) {
    // good place to steal from: https://github.com/REVrobotics/2025-REV-ION-FRC-Starter-Bot/blob/main/src/main/java/frc/robot/subsystems/CoralSubsystem.java
    // also see https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java
    if (goal > lastGoal) {
      wheresItGoin = ElevatorConstants.WaysItCanMove.up;
    } else {
      wheresItGoin = ElevatorConstants.WaysItCanMove.down;
      //TODO it doesnt change to "nowhere" when it reached the goal
      //Also theres no way to see when its done
    }
    lastGoal = goal;
    m_controller.setGoal(goal);
    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_encoder.getDistance());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput + feedforwardOutput);
    //TODO remove this 0.1 magic number cause idk what it is
    m_elevatorSim.setState(m_encoderSim.getDistance(), 0.1);
    //teto
    SmartDashboard.putNumber("Position meters", m_elevatorSim.getPositionMeters());
    SmartDashboard.putString("Movement", wheresItGoin.toString());
  }

  /**
   * Goes to a place
   * corrie said the safety is going outside of this so this doenst check for that
   * @param goal where to go, IDK the units!!
   */
  public void reachGoal(double goal) {
    reachGoal(goal, 1);
    //it looks like recursion but its not cause java is weird
  }

  /** Stop the control loop and motor output. 
   * WARNIG if you called a method before it'll stop in the middle!!!
  */
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


  /**
   * goes either branch 2 3 or 4
   * @param level witch branch
   */
  public Command toBranch(GameConstants.ReefLevels level) {
    return this.run(()->{
      //mmmmmm switch case
      switch (level) {
        case l2: {
          reachGoal(GameConstants.Positions.reefLevels[2]);
          break;
        }
        case l3: {
          reachGoal(GameConstants.Positions.reefLevels[3]);
          break;
        }
        case l4: {
          reachGoal(GameConstants.Positions.reefLevels[4]);
          break;
        }
      }
    });
  }

  /**
   * go to the bottomish
   */
  public Command toFloorIntake() {
    return this.run(()->
      reachGoal(GameConstants.Positions.floorIntake));
  }
  
  /**
   * go to human coral putting station
   */
  public Command toFeeder() {
    return this.run(()->
      reachGoal(GameConstants.Positions.feeder));
  }
  
  /**
   * go to the coral trough
   */
  public Command toLowCoral() {
    return this.run(()->
      reachGoal(GameConstants.Positions.reefLevels[1]));
  }

  /**
   * Idk what stored is ngl
   */
  public Command toStored() {
    return this.run(()->
      reachGoal(GameConstants.Positions.coralStorage));
  }

  /** go to elevator upper limit */
  public Command toTop() {
    return this.run(
      () -> 
        reachGoal(ElevatorConstants.Measurements.kTopHeight)
      );
  }

  /** go to elevator lower limit */
  public Command toBottom() {
    return this.run(
      () ->
        reachGoal(ElevatorConstants.Measurements.kBottomHeight)
      );
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
