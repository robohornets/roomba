package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
    // kSpeedAt12Volts desired top speed
    public static double MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    
    // 3/4 of a rotation per second max angular velocity
    public static double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // MARK: Coordinates of Hub
    public static Translation2d HUB_RED_POSITION = new Translation2d(11.915, 4.034);

    public static Translation2d HUB_BLUE_POSITION = new Translation2d(4.626, 4.034);

    public static double FIELD_LENGTH_METERS = 16.540988;
}
