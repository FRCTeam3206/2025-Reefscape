package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;
import java.util.function.DoubleSupplier;

@Logged
public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax m_max = new SparkMax(ClimberConstants.kClimberCanId, MotorType.kBrushless);
  private final SparkAbsoluteEncoder m_encoder = m_max.getAbsoluteEncoder();

  private boolean canClimb = false;
  private boolean climbed = false;

  public ClimberSubsystem() {
    m_max.configure(
        Configs.ClimberConfigs.climberConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public double getPosition() {
    return m_encoder.getPosition();
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

  public void stop() {
    m_max.setVoltage(0);
  }

  public void runWithLimits(double power) {
    var position = getPosition();
    if (power < 0 && position > ClimberConstants.kMinLimit) {
      m_max.set(power);
    } else if (power > 0 && position < ClimberConstants.kMaxLimit) {
      m_max.set(power);
    } else {
      m_max.set(0);
    }
  }

  public void deploy() {
    var position = getPosition();
    if (position > ClimberConstants.kMaxLimit) {
      m_max.set(0);
    } else {
      m_max.set(1);
    }
  }

  public void climb() {
    var position = getPosition();
    if (position < ClimberConstants.kClimbMin) {
      m_max.set(0);
      climbed = true;
    } else if (canClimb) {
      m_max.set(-1);
    } else {
      m_max.set(0);
    }
  }

  public Command directControl(DoubleSupplier power) {
    return this.run(() -> runWithLimits(power.getAsDouble()));
  }

  public Command deployCommand() {
    return run(() -> deploy())
        .until(() -> getPosition() >= ClimberConstants.kMaxLimit)
        .andThen(
            () -> {
              canClimb = true;
            });
  }

  public Command climbCommand() {
    return run(() -> climb());
  }

  public void resetClimb() {
    canClimb = false;
    climbed = false;
  }

  public boolean getCanClimb() {
    return canClimb;
  }

  public boolean getClimbed() {
    return climbed;
  }

  public Command stopCommand() {
    return this.run(this::stop);
  }
}
