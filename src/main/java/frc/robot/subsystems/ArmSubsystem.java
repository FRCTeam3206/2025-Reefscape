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

@Logged
public class ArmSubsystem extends SubsystemBase {
  private final SparkMax m_max = new SparkMax(ArmConstants.kArmCANId, MotorType.kBrushless);
  private final AbsoluteEncoder m_encoder = m_max.getAbsoluteEncoder();

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ArmConstants.kMaxVelocity, ArmConstants.kMaxAcceleration));

  private final ArmFeedforward feedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kA);
  double ff = 0.0;

  private final PIDController feedback = new PIDController(ArmConstants.kP, 0, ArmConstants.kD);
  double fb = 0.0;

  // Simulation
  private final DCMotor m_armGearbox = DCMotor.getNEO(1);
  private final SparkMaxSim m_maxSim = new SparkMaxSim(m_max, m_armGearbox);
  private final SparkAbsoluteEncoderSim m_encoderSim = m_maxSim.getAbsoluteEncoderSim();

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          ArmSubConstants.kArmReduction,
          ArmSubConstants.kArmMOI,
          ArmSubConstants.kArmLength,
          ArmSubConstants.kMinAngle,
          ArmSubConstants.kMaxAngle,
          true,
          ArmSubConstants.kMinAngle);

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
    m_armSim.setInput(m_maxSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_armSim.update(0.02);

    m_maxSim.iterate(m_armSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    mechArm.setAngle(getAngle());
  }

  public Rotation2d getAngle() {
    return new Rotation2d(m_encoder.getPosition());
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public void moveToGoal(Rotation2d goal) {
    this.goal = new TrapezoidProfile.State(goal.getRadians(), 0);
    this.setpoint = profile.calculate(0.020, this.setpoint, this.goal);
    ff = feedforward.calculate(setpoint.position, setpoint.velocity);
    fb = feedback.calculate(getAngle().getRadians(), setpoint.position);

    m_max.setVoltage(fb + ff);
  }

  public void stop() {
    m_max.setVoltage(0);
  }

  public void reset() {
    setpoint = new TrapezoidProfile.State(getAngle().getRadians(), getVelocity());
  }

  public Command moveToGoalCommand(Rotation2d goal) {
    return runOnce(this::reset).andThen(run(() -> moveToGoal(goal)));
  }

  public Command stopCommand() {
    return this.run(this::stop);
  }
}
