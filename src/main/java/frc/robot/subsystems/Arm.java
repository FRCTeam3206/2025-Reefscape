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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GameConstants;

public final class Arm extends SubsystemBase implements AutoCloseable {
  // This is the arm gearbox
  private final DCMotor m_armGearbox = DCMotor.getNEO(1);

  /// Motors/encoders
  // the encoderssssss
  private final SparkMax m_armMotor = new SparkMax(ArmConstants.kArmCANId, MotorType.kBrushless);
  private final SparkClosedLoopController m_closedLoopController =
      m_armMotor.getClosedLoopController();
  private final SparkAbsoluteEncoder m_absoluteEncoder = m_armMotor.getAbsoluteEncoder();

  private final SparkAbsoluteEncoderSim m_armEncoderSim = new SparkAbsoluteEncoderSim(m_armMotor);
  private final SparkMaxSim m_armMotorSim = new SparkMaxSim(m_armMotor, m_armGearbox);

  /// Simulation classes
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(m_armGearbox, 1, 2, 10, 0, 90, true, 45);

  // TODO: Use these for sim, but right now they are using elevator constants, and we want the robot
  // to work in real life (but it would be great to have sim too)
  // // TODO move mech visualization to the overarching class
  // // Create a Mechanism2d visualization of the elevator
  // private final Mechanism2d m_mech2d =
  //     new Mechanism2d(ElevatorConstants.Mechanism2d.kWidth,
  // ElevatorConstants.Mechanism2d.kHeight);
  // private final MechanismRoot2d m_mech2dRoot =
  //     m_mech2d.getRoot(
  //         "Arm Root",
  //         ElevatorConstants.Mechanism2d.kXDistance,
  //         ElevatorConstants.Mechanism2d.kYDistance);
  // private final MechanismLigament2d m_armMech2d =
  //     m_mech2dRoot.append(new MechanismLigament2d("Arm", m_armSim.getAngleRads(), 90));

  /** Subsystem constructor. */
  public Arm() {
    m_armMotor.configure(
        Configs.Arm.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setGoalPosition(double goal) {
    m_closedLoopController.setReference(goal, ControlType.kMAXMotionVelocityControl);
  }

  public Command setGoalCommand(double goal) {
    return this.run(() -> setGoalPosition(goal));
  }

  public Command toHorizontal() {
    return setGoalCommand(ArmConstants.Angles.kHorizontal);
  }

  public Command toStored() {
    return setGoalCommand(ArmConstants.Angles.kStored);
  }

  public Command toFloorIntake() {
    return setGoalCommand(ArmConstants.Angles.kFloorIntake);
  }

  public Command toFeeder() {
    return setGoalCommand(ArmConstants.Angles.kFeeder);
  }

  public Command toBranch(GameConstants.Levels level) {
    double goal = 0.0;
    switch (level) {
      case l1:
        goal = ArmConstants.Angles.kReefL1;
        break;
      case l2:
        goal = ArmConstants.Angles.kReefL2;
        break;
      case l3:
        goal = ArmConstants.Angles.kReefL3;
        break;
      case l4:
        goal = ArmConstants.Angles.kReefL4;
        break;
    }
    return setGoalCommand(goal);
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    // and non-radial distances
    m_armSim.setInput(m_armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Arm Motor Output", m_armMotorSim.getAppliedOutput());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(ArmConstants.kUpdateFrequency);

    // Required to keep a SparkMax working
    m_armMotorSim.iterate(
        // Multiply by reduction
        Units.radiansPerSecondToRotationsPerMinute(
            m_armSim.getVelocityRadPerSec() * ArmConstants.gearing),
        RoboRioSim.getVInVoltage(),
        ArmConstants.kUpdateFrequency);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    m_armEncoderSim.setPosition(m_armSim.getAngleRads());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
  }

  // Update telemetry
  public void periodic() {
    // m_armMech2d.setAngle(m_absoluteEncoder.getPosition());
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal) {
    m_closedLoopController.setReference(goal, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void close() {
    m_armMotor.close();
  }
}
