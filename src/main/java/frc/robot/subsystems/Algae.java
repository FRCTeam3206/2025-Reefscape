package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class Algae extends SubsystemBase {
  private final SparkMax m_armMotor = new SparkMax(AlgaeConstants.kArmCanId, MotorType.kBrushless);
  private final SparkAbsoluteEncoder m_armAbsoluteEncoder = m_armMotor.getAbsoluteEncoder(); 
  private final SparkMax m_wheelsMotor =
      new SparkMax(AlgaeConstants.kWheelsCanId, MotorType.kBrushless);
  private final PIDController armPID = new PIDController(AlgaeConstants.kArmProportional, AlgaeConstants.kArmIntegral, AlgaeConstants.kArmDerivative);

  private final SparkMaxSim m_armMotorSim =
      new SparkMaxSim(m_armMotor, AlgaeConstants.kArmMotorType);

  public Algae() {}

  @Override
  public void simulationPeriodic() {}

  public void setWheelSpeed(double speed) {
    m_wheelsMotor.set(speed);
  }

  public Command intakeCommand() {
    return this.run(() -> setWheelSpeed(0.1));
  }

  public Command extakeCommand() {
    return this.run(() -> setWheelSpeed(-0.1));
  }

  public double getArmAngle(){
    return m_armAbsoluteEncoder.getPosition();
  }

  public Command retractCommand() {
    return this.run(() -> {
      m_armMotor.set(armPID.calculate(getArmAngle(), AlgaeConstants.kRetractPosition));
    });
  }

  public Command extendCommand() {
    return this.run(() -> {
      m_armMotor.set(armPID.calculate(getArmAngle(), AlgaeConstants.kExtendPosition));
    });
  }
}