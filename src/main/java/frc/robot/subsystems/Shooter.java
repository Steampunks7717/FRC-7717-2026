package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    SparkMax motorPass = new SparkMax(11, MotorType. kBrushless);
    SparkMax motorShooter = new SparkMax(13, MotorType. kBrushless);
    SparkMax motorTravel = new SparkMax(14, MotorType. kBrushless);

    public Shooter(){
    }

    public void shoot() {
        motorTravel.set(-1);
        motorPass.set(0.5);  
        motorShooter.set(1);
    }
    public void shooterpass(){
        motorPass.set(0.5);    
    }
        public void travel() {
        motorTravel.set(-0.6);
   
        }
    public void shooterStop() {
        motorPass.set(0);
        motorShooter.set(0);
        motorTravel.set(0);

}
    public void shootrevert(){
        motorTravel.set(1);
        motorPass.set(-0.3);  
}

     public void shootauto(){
        motorTravel.set(-1);
        motorPass.set( 0.5);
        motorShooter.set(0.9);
    }
}