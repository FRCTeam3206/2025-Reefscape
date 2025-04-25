package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class Algae2 extends SubsystemBase {
    private SparkMax algaeWheels = new SparkMax(AlgaeConstants.kWheelsCanId, MotorType.kBrushless);
    public Algae2() { }
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
    

}

