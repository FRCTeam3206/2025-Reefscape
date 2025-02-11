package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class Coral extends SubsystemBase {
  private final SparkMax m_wheels =
      new SparkMax(CoralConstants.kCANId, MotorType.kBrushless);
  private final DigitalInput m_coralSensor = new DigitalInput(CoralConstants.kSensorChannel);

  public Coral() {}

  public void intake() {
    m_wheels.set(CoralConstants.kIntakeSpeed);
  }

  public void outake() {
    m_wheels.set(CoralConstants.kOutakeSpeed);
  }

  public void stop() {
    m_wheels.set(0);
  }

  public Command intakeCommand() {
    return this.run(() -> intake());
  }

  public Command intakeUntilSuccessCommand() {
    return this.run(() -> intake()).until(() -> m_coralSensor.get());
  }

  public Command scoreCommand() {
    return this.run(() -> intake()).until(() -> !m_coralSensor.get())
        .andThen(this.run(() -> intake()).withTimeout(CoralConstants.kSafeScoreTime));
  }

  public Command outakeCommand() {
    return this.run(() -> outake());
  }

  public Command stopCommand() {
    return this.run(() -> stop());
  }

}
