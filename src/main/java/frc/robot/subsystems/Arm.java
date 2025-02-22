// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Also credit to this repo here most of it is copied n pasted from this
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Robot;

@Logged
public final class Arm extends SubsystemBase implements AutoCloseable {
  private double angle = 0.0;
  private double lastAngle = 0.0;
  private double velocity = 0.0;

  private LinearFilter velocitySmoother = LinearFilter.singlePoleIIR(0.080, 0.020);

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ArmConstants.kMaxVelocity, ArmConstants.kMaxAcceleration));
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          ArmConstants.kS, ArmConstants.kG, ArmConstants.kG, ArmConstants.kA);

  double ff = 0.0;

  private final PIDController feedback =
      new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

  double fb = 0.0;

  /// Motors/encoders
  // the encoderssssss
  private final SparkMax m_armMotor = new SparkMax(ArmConstants.kArmCANId, MotorType.kBrushless);
  private final SparkAbsoluteEncoder m_absoluteEncoder = m_armMotor.getAbsoluteEncoder();

  
  // // TODO: Simulation
  // private final PWMSparkMax motorSim;
  // private final DutyCycleEncoder dcEncoder;
  // private final DutyCycleEncoderSim dcEncoderSim;

  // private final SingleJointedArmSim armSim =
  //     new SingleJointedArmSim(
  //         DCMotor.getNEO(1),
  //         ArmSubConstants.kArmReduction,
  //         ArmSubConstants.kArmMOI,
  //         ArmSubConstants.kArmLength,
  //         ArmSubConstants.kMinAngleRads,
  //         ArmSubConstants.kMaxAngleRads,
  //         true,
  //         ArmSubConstants.kMinAngleRads);

  // @Log
  // private final Mechanism2d mech2d =
  //     new Mechanism2d(3 * ArmSubConstants.kArmRealLength, 3 * ArmSubConstants.kArmRealLength);

  // private final MechanismRoot2d mechArmPivot =
  //     mech2d.getRoot(
  //         "Pivot", 1.5 * ArmSubConstants.kArmRealLength, ArmSubConstants.kArmPivotHeight);

  // @SuppressWarnings("unused")
  // private final MechanismLigament2d mechArmTower =
  //     mechArmPivot.append(
  //         new MechanismLigament2d(
  //             "Tower", 1.5 * ArmConstants.kArmPivotHeight, -90, 12, new Color8Bit(Color.kBlue)));

  // private final MechanismLigament2d mechArm =
  //     mechArmPivot.append(
  //         new MechanismLigament2d(
  //             "Arm",
  //             ArmSubConstants.kArmRealLength,
  //             Units.radiansToDegrees(armSim.getAngleRads()),
  //             6,
  //             new Color8Bit(Color.kYellow)));

  public Arm() {
    m_armMotor.configure(
        Configs.CoralArm.coralArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // if (Robot.isReal()) {
    //   motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    //   motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

      // encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
      // encoder.setZeroOffset(ArmSubConstants.kArmZeroRads);
      // encoder.setPositionConversionFactor(ArmSubConstants.kPositionConversionFactor);
      // encoder.setAverageDepth(16);

      // TODO: simulation
      // // not used for real robot
      // motorSim = null;
      // dcEncoder = null;
      // dcEncoderSim = null;
    // } else {
      // TODO: simulation
      // motorSim = new PWMSparkMax(5);
      // dcEncoder = new DutyCycleEncoder(5);
      // dcEncoderSim = new DutyCycleEncoderSim(dcEncoder);

      // // seed the encoder to have the correct sim arm starting position
      // armSim.update(0.020);
      // dcEncoderSim.setAbsolutePosition(armSim.getAngleRads());

      // // not used for simulation
      // motor = null;
      // encoder = null;
    //}

    feedback.enableContinuousInput(0, 2 * Math.PI);

    angle = getAngle();
    lastAngle = angle;
    this.goal = new TrapezoidProfile.State(angle, 0);
    this.setpoint = new TrapezoidProfile.State(angle, 0);
  }

  public Command setVoltageCommand(DoubleSupplier voltage) {
    return this.run(() -> m_armMotor.set(voltage.getAsDouble()));
  }

  public Command stopCommand() {
    return this.run(() -> m_armMotor.set(0));
  }

  @Override
  public void periodic() {
    super.periodic();
    angle = getAngle();
    velocity = velocitySmoother.calculate((angle - lastAngle) / 0.020);
    // velocity = (angle - lastAngle) / 0.020;
    lastAngle = angle;

    // the mechanism should track the real or simulated arm position
    // TODO: sim
    // mechArm.setAngle(Units.radiansToDegrees(angle));

    // log items that can't be annotated
    // this.log("Setpoint Position", setpoint.position);
    // this.log("Setpoint Velocity", setpoint.velocity);
    // this.log("Goal", this.goal.position);
    // this.log("Error", this.setpoint.position - angle);
    // this.log(
    //     "Voltage",
    //     ((Robot.isReal()) ? motor.getAppliedOutput() : motorSim.get())
    //         * RobotController.getBatteryVoltage());
    // this.log("Current", (Robot.isReal()) ? motor.getOutputCurrent() : armSim.getCurrentDrawAmps());
  }

  public double getAngle() {
    return ((2 * Math.PI * (m_absoluteEncoder.getPosition())) + Math.PI) % (2 * Math.PI);
  }

  public void moveToGoal(double goal) {
    this.goal =
        new TrapezoidProfile.State(goal, 0); // goal is the desired endpoint with zero velocity
    this.setpoint = profile.calculate(0.020, this.setpoint, this.goal);
    ff = feedforward.calculate(setpoint.position, setpoint.velocity);
    fb = feedback.calculate(getAngle(), setpoint.position);

    m_armMotor.set(ff + fb);
  }

  public double getVoltage() {
    return m_armMotor.getAppliedOutput();
  }

  public double getCurrent() {
    return m_armMotor.getOutputCurrent();
  }

  public void reset() {
    setpoint = new TrapezoidProfile.State(getAngle(), velocity);
    feedback.reset();
  }

  /**
   * Creates a command that will move the arm from its current location to the desired goal and hold
   * it there. This command doesn't exit.
   *
   * @param goal the target angle (radians)
   * @return Command that moves the arm and holds it at goal
   */
  public Command moveToGoalCommand(double goal) {
    return runOnce(this::reset).andThen(run(() -> moveToGoal(goal)));
  }

  /**
   * Creates a command that will move the arm from its current location to the desired goal. This
   * command ends when the arm reaches the goal.
   *
   * @param goal the target angle (radians)
   * @return Command that moves the arm and ends when it reaches the goal
   */
  public Command moveToGoalAndStopCommand(double goal) {
    return moveToGoalCommand(goal).until(this::atGoal);
  }

  /**
   * Returns true when the arm is at its current goal and not moving. Tolerances for position and
   * velocity are set in ArmConnstants.
   *
   * @return at goal and not moving
   */
  public boolean atGoal() {
    return MathUtil.isNear(
            this.goal.position, getAngle(), ArmConstants.kAtAngleTolerance, 0, 2 * Math.PI)
        && MathUtil.isNear(0, getVelocity(), ArmConstants.kAtVelocityTolerance);
  }

  public double getVelocity() {
    // double velocity = (Robot.isReal()) ? encoder.getVelocity() : armSim.getVelocityRadPerSec();
    return velocity;
  }

  public Command toHorizontal() {
    return moveToGoalCommand(ArmConstants.Angles.kHorizontal);
  }

  public Command toStored() {
    return moveToGoalCommand(ArmConstants.Angles.kStored);
  }

  public Command toFloorIntake() {
    return moveToGoalCommand(ArmConstants.Angles.kFloorIntake);
  }

  public Command toFeeder() {
    return moveToGoalCommand(ArmConstants.Angles.kFeeder);
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
    return moveToGoalCommand(goal);
  }

  // /** Advance the simulation. */
  // public void simulationPeriodic() {
  //   // In this method, we update our simulation of what our elevator is doing
  //   // First, we set our "inputs" (voltages)
  //   // and non-radial distances
  //   m_armSim.setInput(m_armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
  //   SmartDashboard.putNumber("Arm Motor Output", m_armMotorSim.getAppliedOutput());

  //   // Next, we update it. The standard loop time is 20ms.
  //   m_armSim.update(ArmConstants.kUpdateFrequency);

  //   // Required to keep a SparkMax working
  //   m_armMotorSim.iterate(
  //       // Multiply by reduction
  //       Units.radiansPerSecondToRotationsPerMinute(
  //           m_armSim.getVelocityRadPerSec() * ArmConstants.gearing),
  //       RoboRioSim.getVInVoltage(),
  //       ArmConstants.kUpdateFrequency);

  //   // Finally, we set our simulated encoder's readings and simulated battery
  //   // voltage
  //   m_armEncoderSim.setPosition(m_armSim.getAngleRads());

  //   // SimBattery estimates loaded battery voltages
  //   RoboRioSim.setVInVoltage(
  //       BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
  // }

  // // Update telemetry
  // public void periodic() {
  //   // m_armMech2d.setAngle(m_absoluteEncoder.getPosition());
  // }

  @Override
  public void close() {
    m_armMotor.close();
  }
}
