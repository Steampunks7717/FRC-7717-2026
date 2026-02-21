package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase{
    SparkMax motorIntake = new SparkMax(13, MotorType. kBrushless);

    public Intake(){

    }

        

    public void intake1() {
        motorIntake.set(0.5);
    }
    public void intakeStop() {
        motorIntake.set(0);
}
}