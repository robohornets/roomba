package frc.robot.subsystems.vision.limelight;

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
            0.07, 
            0.07, 
            9999999
    );

    public static final Matrix<N3, N1> QUEST_UPDATE_VISION_STD_DEVS = 
        VecBuilder.fill(
            0.07, 
            0.07, 
            9999999
    );
    
    // Maximum allowed rate of angular rotation before discarding results for innacuracy
    public static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 720.0;

    // Position of Limelight relative to the centre of the robot in metres
    public static final Transform2d LIMELIGHT_4_TRANSFORM_FROM_CENTRE = new Transform2d(
        new Translation2d(0.3429, 0.0), 
        new Rotation2d(0)
    );

    public static final Transform2d LIMELIGHT_2_TRANSFORM_FROM_CENTRE = new Transform2d(
        new Translation2d(0.0, 0.6858), 
        new Rotation2d(1.5 * Math.PI)
    );

    public static Transform2d getTransformForLimelight(String limelightName) {
        if (limelightName.equals("limelight-two")) {
            return LIMELIGHT_2_TRANSFORM_FROM_CENTRE;
        }
        else {
            return LIMELIGHT_4_TRANSFORM_FROM_CENTRE;
        }
    }

    public static Matrix<N3, N1> calculateQuestUpdateStdDevs(PoseEstimate estimate) {
        double xyStdDev = 0.05;
        double thetaStdDev = 9999999;

        // Increase std devs with distance for less trust
        double distanceFactor = Math.max(1.0, estimate.avgTagDist / 2.0);
        xyStdDev *= distanceFactor;

        // Decrease std devs with more tags for more trust
        if (estimate.tagCount >= 2) {
            xyStdDev*= 0.7;
        }

        // Increase std devs with smaller tags for less trust
        if (estimate.avgTagArea < 0.3) {
            xyStdDev *= 1.5;
        }

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
