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
    // 🏳️‍⚧️ TRANS-form
    public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
        new Translation3d(0.0, -0.4, 0.45),
        new Rotation3d(new Rotation2d(-0.5 * Math.PI))
    );

    /**
     * Standard deviations for odometry measurements (wheel encoders + gyro).
     * Lower values = more trust in odometry.
     * Format: [x (meters), y (meters), theta (radians)]
     */
    public static final Matrix<N3, N1> ODOMETRY_STD_DEVS =
        VecBuilder.fill(
            0.05,  // 5cm standard deviation in X
            0.05,  // 5cm standard deviation in Y
            0.01   // ~0.57 degrees standard deviation in rotation
    );

    /**
     * Standard deviations for QuestNav vision measurements.
     * Lower values = more trust in QuestNav.
     * Format: [x (meters), y (meters), theta (radians)]
     */
    public static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2deg/0.035rad rotational
    );

    public static final boolean USE_LIMELIGHT_FOR_VISION_MEASUREMENTS = true;
}
