package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
    SparkMax motorClimber = new SparkMax(15, MotorType.kBrushless);
    public Climber(){

    }
public void ClimbUp() {
        motorClimber.set(1);
    }
public void ClimbDown() {
        motorClimber.set(-1);
    }
    public void ClimberStop() {
        motorClimber.set(0);
}
}