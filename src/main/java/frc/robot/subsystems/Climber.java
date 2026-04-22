package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final SparkMax motorClimber;

    public Climber() {
        if (RobotBase.isReal()) {
            motorClimber = new SparkMax(15, MotorType.kBrushless);
        } else {
            motorClimber = null;
        }
    }

    public void ClimbUp() {
        if (motorClimber == null) return;
        motorClimber.set(1);
    }

    public void ClimbDown() {
        if (motorClimber == null) return;
        motorClimber.set(-1);
    }

    public void ClimberStop() {
        if (motorClimber == null) return;
        motorClimber.set(0);
    }
}
