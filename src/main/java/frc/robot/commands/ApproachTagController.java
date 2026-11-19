package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ApproachTagConstants;

public final class ApproachTagController {
  public record Output(double forward, double rotation, boolean atGoal) {}

  private ApproachTagController() {}

  public static Output calculate(double distanceMeters, double txDegrees) {
    double distanceError = distanceMeters - ApproachTagConstants.kTargetDistanceMeters;
    boolean atDistance = Math.abs(distanceError) < ApproachTagConstants.kDistanceToleranceMeters;
    boolean aligned = Math.abs(txDegrees) < ApproachTagConstants.kAngleToleranceDegrees;

    double forward = atDistance ? 0.0 : output(ApproachTagConstants.kDistanceP * distanceError);
    double rotation = aligned ? 0.0 : output(-ApproachTagConstants.kRotationPPerDegree * txDegrees);
    return new Output(forward, rotation, atDistance && aligned);
  }

  private static double output(double command) {
    double magnitude = MathUtil.clamp(
        Math.abs(command), ApproachTagConstants.kMinOutput, ApproachTagConstants.kMaxOutput);
    return Math.copySign(magnitude, command);
  }
}
