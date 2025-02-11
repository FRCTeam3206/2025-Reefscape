package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.WristConstants;

@Logged
public class Wrist extends SubsystemBase {
  private boolean m_goalHorizontal = true;
  private SparkMax m_wristMotor = new SparkMax(WristConstants.kCANId, MotorType.kBrushless);
  private SparkClosedLoopController m_wristController = m_wristMotor.getClosedLoopController();
  private AbsoluteEncoder m_wristEncoder = m_wristMotor.getAbsoluteEncoder();

  public Wrist() {}

  public Command toVerticalContinuous() {
    return this.run(() -> m_goalHorizontal = false);
  }

  public Command toHorizontalContinuous() {
    return this.run(() -> m_goalHorizontal = true);
  }

  public double getAngle() {
    return m_wristEncoder.getPosition();
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
    m_wristController.setReference(
        m_goalHorizontal ? WristConstants.kHorizontalPosition : WristConstants.kVerticalPosition,
        ControlType.kMAXMotionPositionControl);
  }
}
