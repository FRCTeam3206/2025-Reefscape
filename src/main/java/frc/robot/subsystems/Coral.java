package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class Coral extends SubsystemBase {
  private final SparkMax m_coralLeft =
      new SparkMax(CoralConstants.kFirstMotorCanId, MotorType.kBrushless);
  private final SparkMax m_coralRight =
      new SparkMax(CoralConstants.kSecondMotorCanId, MotorType.kBrushless);

  public Coral() {}

  public void intake() {
    m_coralLeft.set(CoralConstants.kLeftIntakeSpeed);
    m_coralRight.set(CoralConstants.kRightIntakeSpeed);
  }

  public void outake() {
    m_coralLeft.set(CoralConstants.kLeftOutakeSpeed);
    m_coralRight.set(CoralConstants.kRightOutakeSpeed);
  }

  public void moveLeft() {
    m_coralLeft.set(CoralConstants.kLeftIntakeSpeed);
    m_coralRight.set(0);
  }

  public void moveRight() {
    m_coralRight.set(CoralConstants.kRightIntakeSpeed);
    m_coralLeft.set(0);
  }

  public void stop() {
    m_coralRight.set(0);
    m_coralLeft.set(0);
  }

  public Command intakeCommand() {
    return this.run(() -> intake());
  }

  public Command outakeCommand() {
    return this.run(() -> outake());
  }

  public Command moveLeftCommand() {
    return this.run(() -> moveLeft());
  }

  public Command moveRightCommand() {
    return this.run(() -> moveRight());
  }

  public Command stopCommand() {
    return this.run(() -> stop());
  }

}
