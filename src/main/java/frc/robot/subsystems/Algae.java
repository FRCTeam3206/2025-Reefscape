package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class Algae extends SubsystemBase {
  private final SparkMax m_armMotor = new SparkMax(AlgaeConstants.kArmCanId, MotorType.kBrushless);
  private final SparkMax m_wheelsMotor =
      new SparkMax(AlgaeConstants.kWheelsCanId, MotorType.kBrushless);

  private final SparkMaxSim m_armMotorSim =
      new SparkMaxSim(m_armMotor, AlgaeConstants.kArmMotorType);
  private final SparkMaxSim m_wheelsMotorSim =
      new SparkMaxSim(m_wheelsMotor, AlgaeConstants.kWheelsMotorType);

  public Algae() {}

  @Override
  public void simulationPeriodic() {}

  public void setWheelSpeed(double speed) {
    m_wheelsMotor.set(speed);
  }

  public Command intakeCommand() {
    return this.run(() -> setWheelSpeed(0));
  }
}
