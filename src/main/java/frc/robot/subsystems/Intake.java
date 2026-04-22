package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final SparkMax motorIntake;

    public Intake() {
        if (RobotBase.isReal()) {
            motorIntake = new SparkMax(10, MotorType.kBrushed);
        } else {
            motorIntake = null;
        }
    }

    public void intake1() {
        if (motorIntake == null) return;
        motorIntake.set(1);
    }

    public void intakeStop() {
        if (motorIntake == null) return;
        motorIntake.set(0);
    }
}
