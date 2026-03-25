package frc.robot.subsystems.mechanisms.shooter;

public class ShooterDataPoint {
    public double distance;
    public double angle;
    public double speed;

    public ShooterDataPoint(
        double distance,
        double angle,
        double speed
    ) {
        this.distance = distance;
        this.angle = angle;
        this.speed = speed;
    }
}
