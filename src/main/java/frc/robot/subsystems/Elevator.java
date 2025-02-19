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
import edu.wpi.first.math.controller.ElevatorFeedforward;
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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameConstants;

public final class Elevator extends SubsystemBase implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox =
      DCMotor.getNEO(ElevatorConstants.Motor.kHowManyInGearbox);

  // the encoderssssss

  /**
   * Elevator motor module uses a conversion factor of {@link
   * ElevatorConstants.Measurements#kDrumRadius}. PID/encoder use meters instead of rotations.
   */
  private final SparkMax m_motor =
      new SparkMax(ElevatorConstants.Motor.kPort, MotorType.kBrushless);

  private final SparkAbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder();
  private final SparkClosedLoopController m_closedLoopController =
      m_motor.getClosedLoopController();

  // i was readin bout elevators and it said feed forward is good, so i might try dis out...
  // Not set up yet really cause its 8:22pm on a tuesday & im finna crashout
  // TODO corrie said we're doin this another way now so uhhh replace this eventually
  private final ElevatorFeedforward m_feedForward =
      new ElevatorFeedforward(
          ElevatorConstants.FeedForward.Ks,
          ElevatorConstants.FeedForward.Kg,
          ElevatorConstants.FeedForward.Kv,
          ElevatorConstants.FeedForward.Ka);

  // Sim classes
  private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);
  private final SparkAbsoluteEncoderSim m_encoderSim = new SparkAbsoluteEncoderSim(m_motor);

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
          // this spacing is so bad. Every time I see it i go blind
          // BUT spotless apply is just gonna put it back to being ugly... smh...
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
    // dis is a variable so it dfoesnt have to be called 100 times
    double currentPosition = m_elevatorSim.getPositionMeters();
    // once its past the "slow down distance", how close it is to the goal
    // 0.1 is very close, 1 means its right at that distance, bigger than
    // 1 means it s not time to slow down yet
    double percentUntilStop =
        Math.abs(currentPosition - lastGoal) / ElevatorConstants.Measurements.kSlowDownDistance;
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    if (Math.random() < 0.001) {
      double randomThingy = Math.random() * 1.25;
      reachGoal(randomThingy);
    }

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(ElevatorConstants.kUpdateFrequency);

    // Required to keep a SparkMax working
    m_motorSim.iterate(
        m_elevatorSim.getVelocityMetersPerSecond(),
        RobotController.getBatteryVoltage(),
        ElevatorConstants.kUpdateFrequency);

    SmartDashboard.putNumber("percent till stop distance", percentUntilStop);

    // if its going down and its below the goal, or its going up and above the goal
    // then it should stop
    if (wheresItGoin == ElevatorConstants.WaysItCanMove.up) {
      // TODO find a better way to check if its at the goal
      // TODO use feedforward and whaetever calc stuff corrie was talking about
      // Calc means calculator for those of yall new in the chat
      if (percentUntilStop < 0.1) {
        // If its 1 centimeter off its fine
        stayInPlace();
      } else if (percentUntilStop < 1) {
        // erm read the next comment to see wut dis does........ YEAH
        changeSpeed(ElevatorConstants.Voltages.kUp * percentUntilStop);
      }
    } else if (wheresItGoin == ElevatorConstants.WaysItCanMove.down) {
      if (percentUntilStop < 0.1) {
        stayInPlace();
      } else if (percentUntilStop < 1) {
        // Makes it go  slower the farther it is from the goal on a striaght curve
        // basically like feed forward but stupider
        // wait... straight... ITS PRI DE MONTH!!! WE CANT DO THAT!!!!
        // i need a queer to get us on some sine waves stat
        changeSpeed(ElevatorConstants.Voltages.kDown * percentUntilStop);
      }
    }

    SmartDashboard.putNumber("Motor output", m_motorSim.getAppliedOutput());
    SmartDashboard.putNumber("Position meters", m_elevatorSim.getPositionMeters());
    SmartDashboard.putNumber("Goal", lastGoal);
    SmartDashboard.putString("Movement", wheresItGoin.toString());

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setPosition(m_elevatorSim.getPositionMeters());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    updateTelemetry();
  }

  /**
   * Goes to a place in meters
   *
   * @param goal where 2 go
   * @throws Error if the goal is bigger than max heigh t plus arm height or less than 0
   */
  public void reachGoal(double goal) {
    // Error handliong
    if (goal > ElevatorConstants.Measurements.kTopHeight + ArmConstants.kHeight
        || goal < ElevatorConstants.Measurements.kBottomHeight) {
      throw new Error(
          // Ugly-ass string concantation
          // is like hiler... and the concantation camp... Very Bad!
          "Goal is out of range! Should be between"
              + ElevatorConstants.Measurements.kTopHeight
              + ArmConstants.kHeight
              + "and "
              + ElevatorConstants.Measurements.kBottomHeight
              + " meters. Attempted goal: "
              + goal);
    }

    // makes it go up or down or no where...
    if (goal < m_elevatorSim.getPositionMeters()) {
      wheresItGoin = ElevatorConstants.WaysItCanMove.down;
      changeSpeed(ElevatorConstants.Voltages.kDown);
    } else if (goal > m_elevatorSim.getPositionMeters()) {
      wheresItGoin = ElevatorConstants.WaysItCanMove.up;
      changeSpeed(ElevatorConstants.Voltages.kUp);
    } else {
      wheresItGoin = ElevatorConstants.WaysItCanMove.nowhere;
      stayInPlace();
    }

    // record of where it was goin last time
    lastGoal = goal;

    // idk what this does tbh
    // Corrie was sayin  its good and shes in like 15th grade math so i trust what she says
    m_feedForward.calculate(0.1);
  }

  /**
   * reference of the motor whatever that means
   *
   * @param voltage How many volts to change it to, it can be negative to😛😛😛
   */
  public void changeSpeed(double voltage) {
    m_closedLoopController.setReference(voltage, ControlType.kVoltage);
  }

  /**
   * makes the elevator motors turn go up but only a little bit so that it stays in place I just
   * remembered we have to carry stuff which is heavy so this probly wont wokr.... goddammit...
   */
  public void stayInPlace() {
    wheresItGoin = ElevatorConstants.WaysItCanMove.nowhere;
    changeSpeed(ElevatorConstants.Voltages.kStatic);
  }

  /**
   * Stop the control loop and motor output. WARNIG if you called a method before it'll stop in the
   * middle!!!
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

  /** makes it stop but a void and no command TODO less shitty name */
  public void stopButNotCommand() {
    wheresItGoin = ElevatorConstants.WaysItCanMove.nowhere;
    SmartDashboard.putString("Movement", wheresItGoin.toString());
    m_closedLoopController.setReference(0, ControlType.kMAXMotionPositionControl);
    m_motor.set(0.0);
    m_motor.stopMotor();
  }

  /*
   * GO to branch
   * Gun to your head name 5 numbers
   * UHHHHHH.... A B C D E
   * @param level witch branch
   */
  public Command toBranch(GameConstants.ReefLevels level) {
    return this.runOnce(
        () -> {
          // mmmmmm switch case
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

  /** Idk what stored is ngl */
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
