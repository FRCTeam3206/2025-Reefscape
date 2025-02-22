package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeConstants;

@Logged
public class Algae extends SubsystemBase {
  private final SparkMax m_armMotor = new SparkMax(AlgaeConstants.kArmCanId, MotorType.kBrushless);
  private final RelativeEncoder m_armEncoder = m_armMotor.getAlternateEncoder();
  private final SparkMax m_wheelsMotor =
      new SparkMax(AlgaeConstants.kWheelsCanId, MotorType.kBrushless);

  private final SparkMaxSim m_armMotorSim =
      new SparkMaxSim(m_armMotor, AlgaeConstants.kArmMotorType);

  public Algae() {
    m_armMotor.configure(
        Configs.Algae.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_wheelsMotor.configure(
        Configs.Algae.wheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void simulationPeriodic() {}

  public double getArmAngle() {
    return m_armEncoder.getPosition();
  }

  public Command intakeCommand() {
    return this.run(() -> m_wheelsMotor.set(AlgaeConstants.kIntakeSpeed));
  }

  public Command extakeCommand() {
    return this.run(() -> m_wheelsMotor.set(AlgaeConstants.kExtakeSpeed));
  }
  
  public Command stopIntakeCommand() {
    return this.run(() -> m_wheelsMotor.set(0));
  }

  public Command retractCommandContinuous() {
    return this.run(() -> m_armMotor.set(AlgaeConstants.kRetractSpeed));
  }

  public Command retractCommand() {
    return retractCommandContinuous().until(() -> getArmAngle() < AlgaeConstants.kRetractedAngle);
  }

  public Command extendCommandContinuous() {
    return this.run(() -> m_armMotor.set(AlgaeConstants.kExtendSpeed));
  }

  public Command extendCommand() {
    return extendCommandContinuous().until(() -> getArmAngle() > AlgaeConstants.kExtendedAngle);
  }

  public Command stopArmCommand() {
    return this.run(() -> m_armMotor.set(0));
  }

  // public double getArmAngle() {
  //   return m_armAbsoluteEncoder.getPosition();
  // }

  // public Command retractCommand() {
  //   return this.run(
  //           () -> {
  //             m_armController.setReference(
  //                 AlgaeConstants.kRetractPosition, ControlType.kMAXMotionPositionControl);
  //           })
  //       .until(() -> atposGoal(AlgaeConstants.kRetractPosition))
  //       .andThen(this.run(() -> m_armMotor.set(0.0)));
  // }

  // public Command extendCommand() {
  //   return this.run(
  //           () -> {
  //             m_armController.setReference(
  //                 AlgaeConstants.kExtendPosition, ControlType.kMAXMotionPositionControl);
  //           })
  //       .until(() -> atposGoal(AlgaeConstants.kExtendPosition))
  //       .andThen(this.run(() -> m_armMotor.set(0.0)));
  // }

  // public boolean atposGoal(double goal) {
  //   return Math.abs(getArmAngle() - goal) < AlgaeConstants.kAtGoalTolerance;
  // }
}
