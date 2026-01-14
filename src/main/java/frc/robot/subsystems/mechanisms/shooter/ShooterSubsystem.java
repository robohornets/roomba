package frc.robot.subsystems.mechanisms.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import java.lang.Math;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // Just the base setup of the class, calculations will be done here (or in this folder) to shoot the fuel

    // TODO: Update enter angle
    private double enterAngle = 65;

    // TODO: VERY IMPORTANT!!!! Put the correct device ids
    public final TalonFX shooterMotor = new TalonFX(-1);

    public double getDistanceToHub(){
        // TODO: calculate distance to hub
        return 5.00;
    }
    
    // TODO: create commands and set motors

    // Distance to hub (X), distance to hub (y), angle of the ball as it enters the hub
    public double[] calculateShooterValues(double distanceToHubX, double distanceToHubY, double enterAngle){
        // https://www.desmos.com/calculator/aavfklnkvh


        enterAngle = enterAngle * Math.PI / 180;
        double outputAngle = Math.atan(2*distanceToHubY/distanceToHubX - Math.tan(enterAngle));

        double launchSpeed = Math.sqrt(( 9.8 * distanceToHubX*distanceToHubX ) / ( 2 * Math.cos(outputAngle)*Math.cos(outputAngle) * ( distanceToHubX * Math.tan(outputAngle) - distanceToHubY ) ));
        
        return new double[]{outputAngle,launchSpeed};
        
    }

    public double calculateRequiredRPM(double desiredSpeed){

        
        return 0.0;
    }




}
