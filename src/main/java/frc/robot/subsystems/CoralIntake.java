package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralConstants;

@Logged
public class CoralIntake extends SubsystemBase {
  private final SparkMax m_wheels = new SparkMax(CoralConstants.kCANId, MotorType.kBrushless);
  private final DigitalInput m_coralSensor = new DigitalInput(CoralConstants.kSensorChannel);

  public CoralIntake() {
    m_wheels.configure(
        Configs.Coral.wheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake() {
    m_wheels.set(CoralConstants.kIntakeSpeed);
  }

  public void outake() {
    m_wheels.set(CoralConstants.kOutakeSpeed);
  }

  public boolean hasCoral() {
    return !m_coralSensor.get();
  }

  // /**
  //  * Sets the position of the finger
  //  *
  //  * @param retained if true it will block the pipe, if false it will free it
  //  */
  // public Command changeFinger(boolean retained) {
  //   return this.runOnce(
  //       () -> {
  //         if (retained) {
  //           m_finger.set(Constants.CoralConstants.Finger.kRetainedPosition);
  //         } else {
  //           m_finger.set(Constants.CoralConstants.Finger.kFreePosition);
  //         }
  //       });
  // }

  public void stop() {
    m_wheels.set(0);
  }

  public Command intakeCommand() {
    return this.run(
        () -> {
          intake();
        });
  }

  public Command intakeUntilSuccessCommand() {
    return intakeCommand().until(() -> hasCoral()).andThen(stopCommand());
  }

  public Command scoreCommand() {
    return this.run(
        () -> {
          m_wheels.set(CoralConstants.kScoreSpeed);
        });
    // return this.run(
    //         () -> {
    //           intake();
    //           changeFinger(false);
    //         })
    //     .until(() -> !m_coralSensor.get())
    //     .andThen(this.run(() -> intake()).withTimeout(CoralConstants.kSafeScoreTime));
  }

  public Command outakeCommand() {
    return this.run(
        () -> {
          outake();
        });
  }

  public Command stopCommand() {
    return this.runOnce(
        () -> {
          stop();
        });
  }

  public double getVoltage() {
    return m_wheels.getAppliedOutput() * m_wheels.getBusVoltage();
  }

  public double getCurrent() {
    return m_wheels.getOutputCurrent();
  }
}
