package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class Algae2 extends SubsystemBase {
    private SparkMax algaeWheels = new SparkMax(AlgaeConstants.kWheelsCanId, MotorType.kBrushless);
    public Algae2() { }
    public void intakeAlgae() {
        algaeWheels.set(0.1);
    }
    public Command intakeAlgaeCommand() {
        return run(()->intakeAlgae()); 
    } 
    public void stopAlgae() {
        algaeWheels.set(0.0);
    }
    public Command stopAlgaeCommand() {
        return run(()->stopAlgae());
    }
    

}

