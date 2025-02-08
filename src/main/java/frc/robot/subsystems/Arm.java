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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public final class Arm extends SubsystemBase implements AutoCloseable {
  // This is the arm gearbox
  private final DCMotor m_armGearbox = DCMotor.getNEO(1);

  /// Motors/encoders
  // the encoderssssss
  private final SparkMax m_armMotor = new SparkMax(19, MotorType.kBrushless);
  private final SparkClosedLoopController m_closedLoopController = m_armMotor.getClosedLoopController();
  private final AbsoluteEncoder m_absoluteEncoder = m_armMotor.getAbsoluteEncoder();

  // private final EncoderSim m_armEncoderSim = new EncoderSim(m_armEncoder);
  private final SparkMaxSim m_armMotorSim = new SparkMaxSim(m_armMotor, m_armGearbox);

  /// Simulation classes
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(m_armGearbox, 1, 2, 10, 0, 90, true, 45);

  // TODO move mech visualization to the overarching class
  // // Create a Mechanism2d visualization of the elevator
  // private final Mechanism2d m_mech2d =
  //     new Mechanism2d(ElevatorConstants.Mechanism2d.kWidth, ElevatorConstants.Mechanism2d.kHeight);
  // private final MechanismRoot2d m_mech2dRoot =
  //     m_mech2d.getRoot(
  //         "Arm Root",
  //         ElevatorConstants.Mechanism2d.kXDistance,
  //         ElevatorConstants.Mechanism2d.kYDistance);
  // private final MechanismLigament2d m_armMech2d;

  /** Subsystem constructor. */
  public Arm() {
    m_armMotor.configure(
      Configs.Arm.armConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  public void setGoalPosition(double goal) {
    m_closedLoopController.setReference(goal, ControlType.kMAXMotionVelocityControl);
  }

  // public Command toDefault() {
  //   return this.run()
  // }

  // public Command toStored() {}

  // public Command toFloorIntake() {}

  // public Command toFeeder() {}

  // public Command toLowCoral() {}

  // public Command toBranch(int level) {}

  // public boolean atGoal() {}

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    // TODO Convert elevatorSim iterate() and armEncoderSim.setDistance to radial
    // and non-radial distances
    m_armSim.setInput(m_armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Arm Motor Output", m_armMotorSim.getAppliedOutput());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(ElevatorConstants.kUpdateFrequency);

    // Required to keep a SparkMax working
    m_armMotorSim.iterate(
        // Multiply by reduction
        Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec() * 10),
        RoboRioSim.getVInVoltage(),
        ElevatorConstants.kUpdateFrequency);
    SmartDashboard.putNumber(
        "vel", Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec()));

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    // m_armEncoderSim.setDistance(m_armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
  }

  public void periodic() {

  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   * @param speed how fast to go 0 (slow) to 1 (fast)
   */
  public void reachGoal(double goal, double speed) {
    m_closedLoopController.setReference(goal, ControlType.kMAXMotionPositionControl);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal where to go, IDK the units!!
   * @return nuthin!
   */
  public void reachGoal(double goal) {
    reachGoal(goal, 1);
    // i hope i dont get stack overflow! !! !! I>mgiht though im stupid
  }

  /** Update telemetry, including the mechanism visualization. */
  public double returnAngle() {
    // Update elevator visualization with position
    return m_absoluteEncoder.getPosition() * 360;
  }

  @Override
  public void close() {
    m_armMotor.close();
  }
}
