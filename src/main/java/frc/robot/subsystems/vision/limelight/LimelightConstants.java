package frc.robot.subsystems.vision.limelight;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.limelight.LimelightHelpers.PoseEstimate;

public class LimelightConstants {
    // Standard deviations or sexually transmitted disease developments?
    public static final Matrix<N3, N1> VISION_STD_DEVS = 
        VecBuilder.fill(
            0.1, 
            0.1, 
            9999999
    );
    
    // Maximum allowed rate of angular rotation before discarding results for innacuracy
    public static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 720.0;

    // Position of Limelight relative to the centre of the robot in metres
    public static final Transform2d LIMELIGHT_4_TRANSFORM_FROM_CENTRE = new Transform2d(
        new Translation2d(0.0762, -0.26035), 
        new Rotation2d(-0.174532925199433)
    );

    /** Calculate dynamic standard deviations */
    public static Matrix<N3, N1> calculateDynamicStdDevs(PoseEstimate estimate) {
        double baseStdDev = 0.2;
        double thetaStdDev = 9999999;

        double xyStdDev = baseStdDev * (estimate.avgTagDist * estimate.avgTagDist) / estimate.tagCount;

        if (estimate.avgTagDist == 1.0) {
            xyStdDev *= 3;
        }

        Logger.recordOutput("Limelight/CalculatedXYStdDev", xyStdDev);
        
        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    /**
     * Checks if a Limelight pose estimate is high-confidence enough to trigger
     * a QuestNav correction.
     *
     * @param estimate the pose estimate from Limelight
     * @return true if the estimate meets high-confidence criteria
     */
    public static boolean isHighConfidenceForQuestNavCorrection(PoseEstimate estimate) {
        if (estimate == null) {
            return false;
        }

        return estimate.tagCount >= 2 &&
               estimate.avgTagDist < 2.5 &&
               estimate.avgTagArea > 0.4;
    }
}
