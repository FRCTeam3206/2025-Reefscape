package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class Coral extends SubsystemBase {
    private final SparkMax m_coralFirst = new SparkMax(CoralConstants.kFirstMotorCanId, MotorType.kBrushless);
    private final SparkMax m_coralSecond = new SparkMax(CoralConstants.kSecondMotorCanId, MotorType.kBrushless);
    
    public Coral(){}

    public void setFirstSpeed(double speed){
        m_coralFirst.set(speed);
    }

    public void setSecondSpeed(double speed){
        m_coralSecond.set(speed);
    }

    public Command setFirstSpeedCommand() {
        return this.run();
    }
    
}

