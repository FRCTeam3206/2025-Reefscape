package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeConstants;

public class Algae2 extends SubsystemBase {
    private SparkMax algaeWheels = new SparkMax(AlgaeConstants.kWheelsCanId, MotorType.kBrushless);
    private SparkMax algaeArm = new SparkMax(AlgaeConstants.kArmCanId, MotorType.kBrushless);
    public Algae2() {
        algaeWheels.configure(Configs.Algae.wheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algaeArm.configure(Configs.Algae.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public Command setArmPowerCommand(double speed) {
        return run(() ->algaeArm.set(speed));    
    }
    public Command moveArmDown() {
        return setArmPowerCommand(AlgaeConstants.kArmDownSpeed);
    }
    public Command moveArmUp() {
        return setArmPowerCommand(AlgaeConstants.kArmUpSpeed);
    }
    public Command setPowerCommand(double speed) {
        return run(()->algaeWheels.set(speed));
    }
    public Command intakeAlgaeCommand() {
        return setPowerCommand(AlgaeConstants.kAlgeaIntakeWheelSpeed);
    } 
    public Command stopAlgaeCommand() {
        return setPowerCommand(0.0);
    }
    public Command extakeCommand() {
        return setPowerCommand(AlgaeConstants.kALgaeExtakeSpeed);
    }
    public Command stopCommand() {
        return run(()->{
            algaeWheels.set(0);
            algaeArm.set(0);
        });
    }
    

}

