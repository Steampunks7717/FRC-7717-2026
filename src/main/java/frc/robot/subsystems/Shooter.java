package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    SparkMax motorShooter = new SparkMax(10, MotorType. kBrushless);
    SparkMax motorTravel = new SparkMax(11, MotorType. kBrushless);
    SparkMax motorPass = new SparkMax(12, MotorType. kBrushless);

    public Shooter(){
        

    }

    public void shoot() {
        motorShooter.set(0.5);
        motorTravel.set(0.5);
        motorPass.set(0.5);
    }
    public void shooterStop() {
        motorShooter.set(0);
        motorTravel.set(0);
        motorPass.set(0);
}
}