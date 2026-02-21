package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class QuestNavConstants {
    // 🏳️‍⚧️ TRANS-form 3d
    public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
        new Translation3d(0.0, -0.4, 0.45),
        new Rotation3d(new Rotation2d(-0.5 * Math.PI))
    );

    /**
     * Standard deviations for odometry measurements (wheel encoders + gyro).
     * Lower values mean more trust in odometry.
     */
    public static final Matrix<N3, N1> ODOMETRY_STD_DEVS =
        VecBuilder.fill(
            0.05,  // 5cm standard deviation in X
            0.05,  // 5cm standard deviation in Y
            0.01   // ~0.57 degrees standard deviation in rotation
    );

    /**
     * Standard deviations for QuestNav vision measurements.
     * Lower values mean more trust in QuestNav.
     */
    public static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.02, // 2cm standard deviation in X
            0.02, // 2cm standard deviation in Y
            0.035 // 2deg/0.035rad standard deviation in rotation
    );

    public static final boolean USE_LIMELIGHT_FOR_VISION_MEASUREMENTS = true;

    /**
     * Minimum time (in seconds) between QuestNav estimates
     */
    public static final double MIN_CORRECTION_INTERVAL = 1.0;
}
