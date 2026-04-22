package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase{
    SparkMax motorIntake = new SparkMax(10, MotorType.kBrushed);

    public Intake(){

    }

    public void intake1() {
        motorIntake.set(1);
    }
    public void intakeStop() {
        motorIntake.set(0);
}
}