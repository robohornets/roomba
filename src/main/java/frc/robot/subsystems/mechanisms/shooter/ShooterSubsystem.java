package frc.robot.subsystems.mechanisms.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.Pigeon2;
import java.lang.Math;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    // TODO: VERY IMPORTANT!!!! Put the correct device ids
    public final TalonFX shooterTopMotor = new TalonFX(-1);
    public final TalonFX shooterBottomMotor = new TalonFX(-2);
    public final TalonFX shooterYawMotor = new TalonFX(-3);
    public final TalonFX shooterPitchMotor = new TalonFX(-4);

    // MARK: Would we need a gyroscope?
    public final Pigeon2 shooterPigeon = new Pigeon2(-5);


    // TODO: Calculate values
    public double enterAngle = -70;

    public double angleSpeed = 0.1;
    public double angleHoldSpeed = 0.02;






    // Distance to hub (X), distance to hub (Y), angle of the ball as it enters the hub
    public double[] calculateShooterValues(double distanceToHubX, double distanceToHubY, double enterAngle){
        // https://www.desmos.com/calculator/aavfklnkvh


        enterAngle = enterAngle * Math.PI / 180;
        double outputAngle = Math.atan(2*distanceToHubY/distanceToHubX - Math.tan(enterAngle));

        double launchSpeed = Math.sqrt(( 9.80665 * distanceToHubX*distanceToHubX ) / ( 2 * Math.cos(outputAngle)*Math.cos(outputAngle) * ( distanceToHubX * Math.tan(outputAngle) - distanceToHubY ) ));
        
        return new double[]{launchSpeed, outputAngle};
        
    }

    public double calculateRequiredRPM(double desiredSpeed){
        // TODO: Do math to calculate rpm (depends on launcher design)

        return 0.0;
    }

    // TODO: create commands and set motors

    // Point shooter at hub
    public Command alignShooterYawToHub(){
        return Commands.run(
            () -> {
                double desiredRotations = calculateRotationsToFace(getHubPosition()); // rotations required to face towards the hub
                double currentRotations = getShooterYaw(); // current rotations

                System.out.println(currentRotations);
                System.out.println(desiredRotations);

                if (Math.abs(currentRotations - desiredRotations) > 0.01){
                    System.out.println("Needs to rotate");
                    if (currentRotations < desiredRotations){ // rotate positive towards desired
                        shooterYawMotor.set(angleSpeed);
                    } else { // rotate negative towards desired
                        shooterYawMotor.set(-angleSpeed);
                    }
                } else {
                    System.out.println("Needs to stop");
                    shooterYawMotor.set(0); // stop the motor once its within 0.01 rotations of the target
                }
            }
        );
    }




    public double calculateRotationsToFace(double[] position){
        return (
            Math.atan2(
                position[1],position[0]
            ) / (2 * Math.PI)
            + 1
        ) % 1;

    }

    public double getShooterPitch(){
        return shooterPitchMotor.getPosition().getValueAsDouble();
    }
    public double getShooterYaw(){
        return shooterYawMotor.getPosition().getValueAsDouble();
    }
    public double getPigeonPitch(){
        return shooterPigeon.getPitch().getValueAsDouble();
    }
    public double getPigeonYaw(){
        return shooterPigeon.getYaw().getValueAsDouble();
    }

    public double[] getHubPosition(){
        // TODO: calculate hub position using limelight (relative to robot)

        return new double[]{5.00, 2.00};
    }


}
