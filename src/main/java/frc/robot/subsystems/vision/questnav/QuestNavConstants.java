package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class QuestNavConstants {
    // TODO: Add x, y, & z offsets for the actual mount on the robot
    public static final Transform3d ROBOT_TO_QUEST = new Transform3d();

    public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = 
        VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2deg/0.35rad rotational
    );
}
