package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
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
import frc.robot.Constants.ArmSubConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

@Logged
public class ArmSubsystem extends SubsystemBase {
  private double simVoltage = 0;
  private final SparkMax m_max = new SparkMax(ArmConstants.kArmCANId, MotorType.kBrushless);
  private final AbsoluteEncoder m_encoder = m_max.getAbsoluteEncoder();

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ArmSubConstants.kMaxVelocity, ArmSubConstants.kMaxAcceleration));

  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          ArmSubConstants.kS, ArmSubConstants.kG, ArmSubConstants.kV, ArmSubConstants.kA);
  double ff = 0.0;

  private final PIDController feedback =
      new PIDController(ArmSubConstants.kP, 0, ArmSubConstants.kD);
  double fb = 0.0;

  // Simulation
  private final DCMotor m_armGearbox = DCMotor.getNEO(1);
  private final SparkMaxSim m_maxSim = new SparkMaxSim(m_max, m_armGearbox);
  private final SparkAbsoluteEncoderSim m_encoderSim = m_maxSim.getAbsoluteEncoderSim();

  public boolean atGoal = false;

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          ArmSubConstants.kArmReduction,
          ArmSubConstants.kArmMOI,
          ArmSubConstants.kArmLength,
          -Math.PI / 6,
          Math.PI / 2 - 0.1,
          true,
          0);

  private final Mechanism2d mech2d =
      new Mechanism2d(3 * ArmSubConstants.kArmLength, 3 * ArmSubConstants.kArmLength);

  private final MechanismRoot2d mechArmPivot =
      mech2d.getRoot("Pivot", 1.5 * ArmSubConstants.kArmLength, ArmSubConstants.kArmPivotHeight);

  private final MechanismLigament2d mechArmTower =
      mechArmPivot.append(
          new MechanismLigament2d(
              "Elevator", 1.5 * ArmSubConstants.kArmLength, -90, 12, new Color8Bit(Color.kBlue)));

  private final MechanismLigament2d mechArm =
      mechArmPivot.append(
          new MechanismLigament2d(
              "Arm",
              ArmSubConstants.kArmLength,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  public ArmSubsystem() {
    m_max.configure(
        Configs.CoralArmSubsystem.coralArmConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    feedback.enableContinuousInput(0, Units.rotationsToRadians(1));
    SmartDashboard.putData("Arm", mech2d);
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    simVoltage = RoboRioSim.getVInVoltage();

    m_armSim.setInputVoltage(m_maxSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_armSim.update(0.02);

    m_maxSim.iterate(
        m_armSim.getVelocityRadPerSec() * 60 / (2 * Math.PI), RoboRioSim.getVInVoltage(), 0.02);
    m_encoderSim.iterate(
        m_armSim.getVelocityRadPerSec() * 60 / (2 * Math.PI) / ArmSubConstants.kArmReduction, 0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    mechArm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }

  public Rotation2d getAngle() {
    if (Robot.isSimulation()) {
      return new Rotation2d(m_encoderSim.getPosition());
    }
    return new Rotation2d(m_encoder.getPosition());
  }

  public Command setVoltageDirectly(DoubleSupplier voltage) {
    return run(() -> m_max.setVoltage(voltage.getAsDouble()));
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public double getAppliedVoltage() {
    return m_max.getAppliedOutput() * m_max.getBusVoltage();
  }

  public double getCurrent() {
    return m_max.getOutputCurrent();
  }

  public double getSetPoint() {
    return setpoint.position;
  }

  public double getSetpointVelocity() {
    return setpoint.velocity;
  }

  public double getGoal() {
    return goal.position;
  }

  public double getArmSimAngle() {
    return m_armSim.getAngleRads();
  }

  public double getArmSimVelocity() {
    return m_armSim.getVelocityRadPerSec();
  }

  public void moveToGoal(Rotation2d goal) {
    var cur_angle = setpoint.position;
    var cur_velocity = setpoint.velocity;
    this.goal = new TrapezoidProfile.State(goal.getRadians(), 0);
    this.setpoint = profile.calculate(0.020, this.setpoint, this.goal);
    ff = feedforward.calculateWithVelocities(cur_angle, cur_velocity, setpoint.velocity);
    fb = feedback.calculate(getAngle().getRadians(), setpoint.position);

    m_max.setVoltage(fb + ff);

    atGoal(goal.getRadians());
  }

  public boolean safeWrist() {
    return getAngle().getRadians() < ArmSubConstants.kSafeWrist;
  }

  /**
   * Returns true when the arm is at its current goal and not moving. Tolerances for position and
   * velocity are set in ArmConstants.
   *
   * @return at goal and not moving
   */
  public boolean atGoal(double goal) {
    atGoal = Math.abs(goal - getAngle().getRadians()) < ArmConstants.kAtAngleTolerance;
    return atGoal;
    // return MathUtil.isNear(
    //         this.goal.position,
    //         getAngle().getRadians(),
    //         ArmConstants.kAtAngleTolerance,
    //         0,
    //         2 * Math.PI)
    //     && MathUtil.isNear(0, getVelocity(), ArmConstants.kAtVelocityTolerance);
  }

  public void stop() {
    m_max.setVoltage(0);
    atGoal = false;
  }

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

  public Command stopCommand() {
    return this.run(this::stop);
  }

  public Command toHorizontal() {
    return moveToGoalCommand(ArmConstants.Angles.kHorizontal);
  }

  public Command toStored() {
    return moveToGoalCommand(ArmConstants.Angles.kStored);
  }

  public Command toStoredStop() {
    return moveToGoalAndStopCommand(ArmConstants.Angles.kStored);
  }

  public boolean isSafe() {
    return getAngle().getRadians() > ArmConstants.Angles.kSafePosition;
  }

  public Command toStoredSafe() {
    return moveToGoalCommand(ArmConstants.Angles.kStored).until(() -> isSafe());
  }

  public Command toFeederIntake() {
    return moveToGoalCommand(ArmConstants.Angles.kFeeder);
  }

  public Command toFloorIntakeStop() {
    return moveToGoalAndStopCommand(ArmConstants.Angles.kFloorIntake).andThen(stopCommand());
    // .until(() -> atGoal())
    // .andThen(stopCommand());
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

  public Command toL2L3() {
    return moveToGoalCommand(ArmConstants.Angles.kReefL2);
  }

  public Command toBranchStop() {
    return moveToGoalAndStopCommand(ArmConstants.Angles.kReefL2);
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
}
