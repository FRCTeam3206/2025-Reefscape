package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public final class Climber extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(ClimberConstants.kCanId, MotorType.kBrushless);

  public Climber() {
    m_motor.configure(
      Configs.Climber.climberConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);
  }

  public Command goOut() {
    return this.run(() -> m_motor.set(ClimberConstants.kSpeed));
  }

  public Command goIn() {
    return this.run(() -> m_motor.set(-ClimberConstants.kSpeed));
  }
}
