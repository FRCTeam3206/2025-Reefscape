package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.WristConstants;

@Logged
public class Wrist extends SubsystemBase {
  private boolean m_goalHorizontal = true;
  private SparkMax m_motor = new SparkMax(WristConstants.kCANId, MotorType.kBrushless);
  // private SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
  private AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder();
  private double m_arbFF = 0.0;

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(WristConstants.kMaxVelocity, WristConstants.kMaxAcceleration));

  private final PIDController feedback = new PIDController(WristConstants.kP, 0, WristConstants.kD);
  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, WristConstants.kV);
  double fb = 0.0;
  double ff = 0.0;

  public Wrist() {
    m_motor.configure(
        Configs.Wrist.wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command toVerticalContinuous() {
    return this.run(() -> m_goalHorizontal = false);
  }

  public Command toHorizontalContinuous() {
    return this.run(() -> m_goalHorizontal = true);
  }

  public Command toVerticalStop() {
    return toVerticalContinuous().until(() -> isVertical().getAsBoolean());
  }

  public Command toHorizontalStop() {
    return toHorizontalContinuous().until(() -> isHorizontal().getAsBoolean());
  }

  public double getAngle() {
    return m_encoder.getPosition();
  }

  public Trigger isVertical() {
    return new Trigger(() -> atAngle(WristConstants.kVerticalPosition));
  }

  public Trigger isHorizontal() {
    return new Trigger(() -> atAngle(WristConstants.kHorizontalPosition));
  }

  public boolean atAngle(double desiredAngle) {
    return Math.abs(getAngle() - desiredAngle) < WristConstants.kAtAngleTolerance;
  }

  public double getVoltage() {
    return m_motor.getAppliedOutput() * m_motor.getBusVoltage();
  }

  public double getCurrent() {
    return m_motor.getOutputCurrent();
  }

  public double getSetpoint() {
    return setpoint.position;
  }

  public double getSetpointVelocity() {
    return setpoint.velocity;
  }

  @Override
  public void periodic() {
    double goal = m_goalHorizontal ? WristConstants.kHorizontalPosition : WristConstants.kVerticalPosition;

    this.goal = new TrapezoidProfile.State(goal, 0);
    this.setpoint = profile.calculate(0.020, this.setpoint, this.goal);

    fb =
        feedback.calculate(
            getAngle(),
            goal);
    ff =
        feedforward.calculate(setpoint.position, setpoint.velocity);

    m_motor.set(fb + ff);
  }
}
