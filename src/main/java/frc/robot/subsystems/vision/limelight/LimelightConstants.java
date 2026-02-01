package frc.robot.subsystems.vision.limelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class LimelightConstants {
    // Standard deviations or sexually transmitted disease developments?
    public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.7, 0.7, 9999999);
    
    // Maximum allowed rate of angular rotation before discarding results for innacuracy
    public static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 720.0;

    // Position of Limelight relative to the centre of the robot in metres
    public static final Transform2d LIMELIGHT_TRANSFORM_FROM_CENTRE = new Transform2d(
        new Translation2d(0.6858, 0.0), 
        new Rotation2d(0)
    );
}
