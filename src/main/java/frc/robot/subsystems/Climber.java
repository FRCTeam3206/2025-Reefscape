package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public final class Climber {
    private final SparkMax motor = new SparkMax(ClimberConstants.kCanId, MotorType.kBrushless);
    public Climber() {

    }
    public void goOut() {

    }
    public void goBackIn() {
        
    }
}
