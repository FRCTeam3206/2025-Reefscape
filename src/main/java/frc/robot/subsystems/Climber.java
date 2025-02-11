package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public final class Climber extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(ClimberConstants.kCanId, MotorType.kBrushless);

  public Climber() {}

  public Command goOut() {
    return this.run(() -> m_motor.set(ClimberConstants.kSpeed));
  }

  public Command goIn() {
    return this.run(() -> m_motor.set(-ClimberConstants.kSpeed));
  }
}
