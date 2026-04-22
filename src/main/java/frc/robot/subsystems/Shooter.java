package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final SparkMax motorPass;
    private final SparkMax motorShooter;
    private final SparkMax motorTravel;

    public Shooter() {
        if (RobotBase.isReal()) {
            motorPass    = new SparkMax(11, MotorType.kBrushless);
            motorShooter = new SparkMax(13, MotorType.kBrushless);
            motorTravel  = new SparkMax(14, MotorType.kBrushless);
        } else {
            motorPass    = null;
            motorShooter = null;
            motorTravel  = null;
        }
    }

    public void shoot() {
        if (motorTravel == null) return;
        motorTravel.set(-1);
        motorPass.set(0.5);
        motorShooter.set(1);
    }

    public void shooterpass() {
        if (motorPass == null) return;
        motorPass.set(0.5);
    }

    public void travel() {
        if (motorTravel == null) return;
        motorTravel.set(-0.6);
    }

    public void shooterStop() {
        if (motorPass == null) return;
        motorPass.set(0);
        motorShooter.set(0);
        motorTravel.set(0);
    }

    public void shootrevert() {
        if (motorTravel == null) return;
        motorTravel.set(1);
        motorPass.set(-0.3);
    }

    public void shootauto() {
        if (motorTravel == null) return;
        motorTravel.set(-1);
        motorPass.set(0.5);
        motorShooter.set(0.9);
    }
}
