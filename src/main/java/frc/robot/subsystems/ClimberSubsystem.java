package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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
  private final RelativeEncoder m_encoder = m_max.getEncoder();

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

  public Command directControl(DoubleSupplier power) {
    return this.run(() -> m_max.set(power.getAsDouble()));
  }

  public Command stopCommand() {
    return this.run(this::stop);
  }
}
