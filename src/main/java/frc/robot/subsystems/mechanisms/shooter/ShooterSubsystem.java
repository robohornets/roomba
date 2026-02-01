package frc.robot.subsystems.mechanisms.shooter;

import java.util.List;

import com.btwrobotics.WhatTime.frc.MotorManagers.MotorWrapper;
import com.btwrobotics.WhatTime.frc.MotorManagers.PositionManager;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.motor.MotorSubsystem;




public class ShooterSubsystem extends SubsystemBase {
    CommandSwerveDrivetrain drivetrain;

    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    MotorSubsystem motorSubsystem = new MotorSubsystem();
    
    public final MotorWrapper shooterMotor = new MotorWrapper(
        new TalonFX(10),
        false
        );
    public final MotorWrapper shooterPitchMotor = new MotorWrapper(
        new TalonFX(9),
        false
    );

    public final Pigeon2 shooterPigeon = new Pigeon2(34);


    // TODO: Calculate values
    public double hubEnterAngle = -70;

    public double shooterPitchSpeed = 0.1;
    public double shooterPitchHoldSpeed = 0.02;

    public double shooterPitchMax = 0.3;
    public double shooterPitchMin = 0.0;

    public double positionThreshold = 1.0; // Threshold for position manager

    public double hubHeight = 2;
    public double shooterHeight = 1;

    public PositionManager shooterPositionManager = new PositionManager(
        shooterPitchMin,
        shooterPitchMax,
        List.of(shooterPitchMotor),
        0.2,
        0.0,
        positionThreshold, 
        () -> getShooterPitchDeg()
    );



    // TODO: create commands and set motors

    public Command pitchToAngleDeg(double angle){
        return shooterPositionManager.move(angle);
    }
        
    public Command aimAtHub(){
        return Commands.run(
            () -> {
                System.out.println("pitchToAngleDeg");
                double[] robotPosition = NetworkTableInstance.getDefault().getTable("CustomDashboard").getEntry("Pose").getDoubleArray(new double[]{0.0,0.0,0.0});
                
                double[] distanceToHub = getDistanceToHub(
                    new double[]{robotPosition[0], robotPosition[1]}
                );

                double degreesToRotate = motorSubsystem.degreesToFace(distanceToHub, robotPosition[2]);
                
                System.out.println(distanceToHub[0]);
                System.out.println(distanceToHub[1]);
                System.out.println(degreesToRotate);


                // rotate the hub by degreesToRotate

    
                double[] launchVariables = motorSubsystem.calculateTrajectory(distanceToHub[0], distanceToHub[1], hubEnterAngle);


                System.out.println(launchVariables[0]);
                System.out.println(launchVariables[1]);
                // pitch towards the required angle


                new PositionManager(
                    shooterPitchMin,
                    shooterPitchMax,
                    Arrays.asList(shooterPitchMotor),
                    launchVariables[1],
                    shooterPitchSpeed,
                    shooterPitchHoldSpeed,
                    positionThreshold,
                    () -> getShooterPitchDeg()
                );
            }
        );
    }



    public double getShooterMotorPitchDeg(){
        return shooterPitchMotor.getPosition() * 360;
    }
    public double getShooterPitchDeg(){
        // MARK: Should be roll?
        return shooterPigeon.getRoll().getValueAsDouble();
    }
    public double[] getDistanceToHub(double[] position){
        // MARK: still in progress
        // meters
        double[] hubPosition = new double[]{12.5, 4.5};
        
        return new double[]{hubPosition[0] - position[0], hubPosition[1] - position[1]};
    }

    // TODO: calculate hub position using limelight (relative to robot) (x,y: front right of robot are positive)

}
