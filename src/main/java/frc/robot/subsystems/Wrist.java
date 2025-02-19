package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.WristConstants;

@Logged
public class Wrist extends SubsystemBase {
  private boolean m_goalHorizontal = true;
  private SparkMax m_motor = new SparkMax(WristConstants.kCANId, MotorType.kBrushless);
  private SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
  private AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder();

  public Wrist() {
    m_motor.configure(
        Configs.Wrist.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command toVerticalContinuous() {
    return this.run(() -> m_goalHorizontal = false);
  }

  public Command toHorizontalContinuous() {
    return this.run(() -> m_goalHorizontal = true);
  }

  public double getAngle() {
    return m_encoder.getPosition();
  }

  public Trigger isVertical() {
    return new Trigger(() -> atAngle(getAngle(), WristConstants.kVerticalPosition));
  }

  public Trigger isHorizontal() {
    return new Trigger(() -> atAngle(getAngle(), WristConstants.kHorizontalPosition));
  }

  public boolean atAngle(double currentAngle, double desiredAngle) {
    return Math.abs(currentAngle - desiredAngle) < WristConstants.kAtAngleTolerance;
  }

  @Override
  public void periodic() {
    m_controller.setReference(
        m_goalHorizontal ? WristConstants.kHorizontalPosition : WristConstants.kVerticalPosition,
        ControlType.kMAXMotionPositionControl);
  }
}
