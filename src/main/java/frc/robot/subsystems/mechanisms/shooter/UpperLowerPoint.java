package frc.robot.subsystems.mechanisms.shooter;

public class UpperLowerPoint {
    public ShooterDataPoint upper;
    public ShooterDataPoint lower;

    public UpperLowerPoint(
        ShooterDataPoint upper,
        ShooterDataPoint lower
    ) {
        this.upper = upper;
        this.lower = lower;
    }

    public double getUpperDistance() {
        return upper.distance;
    }

    public double getLowerDistance() {
        return lower.distance;
    }

    public double getUpperAngle() {
        return upper.angle;
    }

    public double getLowerAngle() {
        return lower.angle;
    }

    public double getUpperSpeed() {
        return upper.speed;
    }

    public double getLowerSpeed() {
        return lower.speed;
    }
}
