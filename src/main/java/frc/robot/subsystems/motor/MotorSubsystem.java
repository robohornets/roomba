package frc.robot.subsystems.motor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;

public class MotorSubsystem extends SubsystemBase {

    public double degreesToFace(double[] position, double angle){ // negative is clockwise positive is counterclockwise
        return angle - (
            -Math.atan2(position[1], position[0])
        ) * 180 / Math.PI;
    }

    public double[] calculateTrajectory(double distanceX, double distanceY, double enterAngle){
        // https://www.desmos.com/calculator/aavfklnkvh


        enterAngle = enterAngle * Math.PI / 180;
        double outputAngle = Math.atan(2 * distanceY / distanceX - Math.tan(enterAngle));

        double launchSpeed = Math.sqrt(
            ( 9.80665 * distanceX * distanceY ) / 
            ( 2 * Math.cos(outputAngle) * Math.cos(outputAngle) 
            * ( distanceX * Math.tan(outputAngle) - distanceY ) 
        ));
        
        return new double[]{launchSpeed, outputAngle * 180 / Math.PI};
        
    }

}
