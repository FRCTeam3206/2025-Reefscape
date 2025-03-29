package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorSubConstants;
import frc.robot.Constants.GameConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Logged
public class Elevator extends SubsystemBase {
  private double simVoltage = 0;
  private final SparkMax m_max =
      new SparkMax(ElevatorConstants.Motor.kCanIdMotor1, MotorType.kBrushless);
  private final SparkMax m_max2 =
      new SparkMax(ElevatorConstants.Motor.kCanIdMotor2, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_max.getEncoder();

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ElevatorSubConstants.kMaxVelocity, ElevatorSubConstants.kMaxAcceleration));

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ElevatorSubConstants.kS,
          ElevatorSubConstants.kG,
          ElevatorSubConstants.kV,
          ElevatorSubConstants.kA);
  double ff = 0.0;

  private final PIDController feedback =
      new PIDController(ElevatorSubConstants.kP, 0, ElevatorSubConstants.kD);
  double fb = 0.0;

  // Simulation
  // private final DCMotor m_armGearbox = DCMotor.getNEO(1);
  // private final SparkMaxSim m_maxSim = new SparkMaxSim(m_max, m_armGearbox);
  // private final SparkAbsoluteEncoderSim m_encoderSim = m_maxSim.getAbsoluteEncoderSim();

  // private final SingleJointedArmSim m_armSim =
  //     new SingleJointedArmSim(
  //         m_armGearbox,
  //         ElevatorSubConstants.kArmReduction,
  //         ElevatorSubConstants.kArmMOI,
  //         ElevatorSubConstants.kArmLength,
  //         -Math.PI / 6,
  //         Math.PI / 2 - 0.1,
  //         true,
  //         0);

  // private final Mechanism2d mech2d =
  //     new Mechanism2d(3 * ElevatorSubConstants.kArmLength, 3 * ElevatorSubConstants.kArmLength);

  // private final MechanismRoot2d mechArmPivot =
  //     mech2d.getRoot("Pivot", 1.5 * ElevatorSubConstants.kArmLength,
  // ElevatorSubConstants.kArmPivotHeight);

  // private final MechanismLigament2d mechArmTower =
  //     mechArmPivot.append(
  //         new MechanismLigament2d(
  //             "Elevator", 1.5 * ElevatorSubConstants.kArmLength, -90, 12, new
  // Color8Bit(Color.kBlue)));

  // private final MechanismLigament2d mechArm =
  //     mechArmPivot.append(
  //         new MechanismLigament2d(
  //             "Arm",
  //             ElevatorSubConstants.kArmLength,
  //             Units.radiansToDegrees(m_armSim.getAngleRads()),
  //             6,
  //             new Color8Bit(Color.kYellow)));

  public Elevator() {
    m_max.configure(
        Configs.ElevatorConfigs.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_max2.configure(
        Configs.ElevatorConfigs.elevatorConfig2,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    // SmartDashboard.putData("Arm", mech2d);
    m_encoder.setPosition(0.0);
    reset();
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  // @Override
  // public void simulationPeriodic() {
  //   super.simulationPeriodic();
  //   simVoltage = RoboRioSim.getVInVoltage();

  //   m_armSim.setInputVoltage(m_maxSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
  //   m_armSim.update(0.02);

  //   m_maxSim.iterate(
  //       m_armSim.getVelocityRadPerSec() * 60 / (2 * Math.PI), RoboRioSim.getVInVoltage(), 0.02);
  //   m_encoderSim.iterate(
  //       m_armSim.getVelocityRadPerSec() * 60 / (2 * Math.PI) /
  // ElevatorSubConstants.kArmReduction, 0.02);

  //   RoboRioSim.setVInVoltage(
  //       BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

  //   mechArm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  // }

  public double getPosition() {
    return m_encoder.getPosition();
    // if (Robot.isSimulation()) {
    //   return new Rotation2d((m_encoderSim.getPosition() + Math.PI) % (2 * Math.PI));
    // }
    // return new Rotation2d((m_encoder.getPosition() + Math.PI) % (2 * Math.PI));
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public double getAppliedVoltage() {
    return m_max.getAppliedOutput() * m_max.getBusVoltage();
  }

  public double getAppliedVoltageFollower() {
    return m_max2.getAppliedOutput() * m_max2.getBusVoltage();
  }

  public double getBusVoltage() {
    return m_max.getBusVoltage();
  }

  public double getBusVoltageFollower() {
    return m_max2.getBusVoltage();
  }

  public double getCurrent() {
    return m_max.getOutputCurrent();
  }

  public double getCurrretFollower() {
    return m_max2.getOutputCurrent();
  }

  public double getSetPoint() {
    return setpoint.position;
  }

  public double getSetPointVelocity() {
    return setpoint.velocity;
  }

  public double getGoal() {
    return goal.position;
  }

  // public double getArmSimAngle() {
  //   return m_armSim.getAngleRads();
  // }

  // public double getArmSimVelocity() {
  //   return m_armSim.getVelocityRadPerSec();
  // }

  public Command setVoltage(DoubleSupplier volts) {
    return run(
        () -> {
          m_max.setVoltage(6 * volts.getAsDouble());
        });
  }

  public void reset() {
    goal = new TrapezoidProfile.State();
    setpoint = new TrapezoidProfile.State();
  }

  public void moveToGoal(double goal) {
    var cur_velocity = this.setpoint.velocity;
    this.goal = new TrapezoidProfile.State(goal, 0);
    this.setpoint = profile.calculate(0.020, this.setpoint, this.goal);
    ff = feedforward.calculateWithVelocities(cur_velocity, setpoint.velocity);
    fb = feedback.calculate(getPosition(), setpoint.position);

    double voltage = fb + ff;
    if (voltage < 0) voltage = 0;
    m_max.setVoltage(voltage);
  }

  public Command moveToGoalCommand(double goal) {
    return run(() -> moveToGoal(goal));
  }

  public Command moveToGoalAndStopCommand(double goal) {
    return moveToGoalCommand(goal).until(() -> atGoal(goal));
  }

  // public Command moveToLevelCommand(double goal) {
  //   return moveToGoalCommand(goal);
  // }

  // public boolean canMoveElevator() {
  //   return getPosition() < 0.005 && getVelocity() == 0;
  // }

  // public Command stayAtLevelCommand(double goal) {
  //   return moveToGoalCommand(goal);
  // }

  // public Command moveToL2Command() {
  //   return moveToLevelCommand(ElevatorSubConstants.kL2Pos);
  // }

  // public Command moveToL3Command() {
  //   return moveToLevelCommand(ElevatorSubConstants.kL3Pos);
  // }

  // public Command moveToL4Command() {
  //   return moveToLevelCommand(ElevatorSubConstants.kL4Pos);
  // }

  /**
   * Returns true when the arm is at its current goal and not moving. Tolerances for position and
   * velocity are set in ArmConstants.
   *
   * @return at goal and not moving
   */
  // public boolean lessThan(double goal) {
  //   return getPosition() + 0.1 < goal; // TODO fix
  //   // return MathUtil.isNear(
  //   //         this.goal.position,
  //   //         getAngle().getRadians(),
  //   //         ArmConstants.kAtAngleTolerance,
  //   //         0,
  //   //         2 * Math.PI)
  //   //     && MathUtil.isNear(0, getVelocity(), ArmConstants.kAtVelocityTolerance);
  // }

  // public boolean lessThanL2() {
  //   return lessThan(ElevatorSubConstants.kL2Pos);
  // }

  // public boolean lessThanL3() {
  //   return lessThan(ElevatorSubConstants.kL3Pos);
  // }

  // public boolean lessThanL4() {
  //   return lessThan(ElevatorSubConstants.kL4Pos);
  // }

  public void stop() {
    m_max.setVoltage(0);
    setpoint = new TrapezoidProfile.State();
  }

  public Command stopCommand() {
    return this.run(this::stop);
  }

  public Command toBranch(GameConstants.ReefLevels level) {
    double goal = 0.0;
    switch (level) {
      case l1:
        return stopCommand();
      case l2:
        goal = ElevatorSubConstants.kL2Pos;
        break;
      case l3:
        goal = ElevatorSubConstants.kL3Pos;
        break;
      case l4:
        goal = ElevatorSubConstants.kL4Pos;
        break;
    }
    return moveToGoalCommand(goal);
  }

  public Command toBranchStop(GameConstants.ReefLevels level) {
    double goal = 0.0;
    switch (level) {
      case l1:
        return stopCommand();
      case l2:
        goal = ElevatorSubConstants.kL2Pos;
        break;
      case l3:
        goal = ElevatorSubConstants.kL3Pos;
        break;
      case l4:
        goal = ElevatorSubConstants.kL4Pos;
        break;
    }
    return moveToGoalAndStopCommand(goal);
  }

  public boolean atGoal(double goal) {
    return MathUtil.isNear(
        goal,
        getPosition(),
        ElevatorSubConstants.kAtGoalTolerance); // Math.abs(getPosition() - goal) <
  }

  public void defaultAction(boolean safeToGoDown) {
    stop();
    // if (safeToGoDown) {
    //   stop();
    // } else {
    //   moveToGoal(getPosition());
    // }
  }

  public Command defaultCommand(BooleanSupplier safeToGoDown) {
    return run(() -> defaultAction(safeToGoDown.getAsBoolean()));
  }

  public Command toFeederCommand() {
    return moveToGoalCommand(ElevatorSubConstants.kFeederPos);
  }

  public Command toFeederCommandStop() {
    return moveToGoalAndStopCommand(ElevatorSubConstants.kFeederPos);
  }

  // public Command stayAtBranch(GameConstants.ReefLevels level) {
  //   double goal = 0.0;
  //   switch (level) {
  //     case l1:
  //       return stopCommand();
  //     case l2:
  //       goal = ElevatorSubConstants.kL2Pos;
  //       break;
  //     case l3:
  //       goal = ElevatorSubConstants.kL3Pos;
  //       break;
  //     case l4:
  //       goal = ElevatorSubConstants.kL4Pos;
  //       break;
  //     default:
  //       return stopCommand();
  //   }
  //   return stayAtLevelCommand(goal);
  // }

  /*
    public void reset() {
      setpoint = new TrapezoidProfile.State(getAngle().getRadians(), getVelocity());
    }

    public Command moveToGoalCommand(Rotation2d goal) {
      return runOnce(this::reset).andThen(run(() -> moveToGoal(goal)));
    }

    public Command moveToGoalCommand(double goal) {
      return moveToGoalCommand(new Rotation2d(goal));
    }

    public Command moveToGoalAndStopCommand(double goal) {
      return moveToGoalCommand(goal).until(() -> atGoal(goal));
    }

    public Command toHorizontal() {
      return moveToGoalCommand(ArmConstants.Angles.kHorizontal);
    }

    public Command toStored() {
      return moveToGoalCommand(ArmConstants.Angles.kStored);
    }

    public Command toFloorIntakeStop() {
      return moveToGoalAndStopCommand(ArmConstants.Angles.kFloorIntake).andThen(stopCommand());
          //.until(() -> atGoal())
          //.andThen(stopCommand());
    }

    public Command toFeeder() {
      return moveToGoalCommand(ArmConstants.Angles.kFeeder);
    }

    public Command toL1() {
      return moveToGoalCommand(ArmConstants.Angles.kReefL1);
    }

    public Command toL1Stop() {
      return moveToGoalAndStopCommand(ArmConstants.Angles.kReefL1);
    }

    public Command toBranch(GameConstants.ReefLevels level) {
      double goal = 0.0;
      switch (level) {
        case l1:
          return toL1();
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
  */
}
