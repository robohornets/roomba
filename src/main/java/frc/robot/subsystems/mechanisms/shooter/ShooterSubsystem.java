package frc.robot.subsystems.mechanisms.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.btwrobotics.WhatTime.frc.PositionManager;
import com.btwrobotics.WhatTime.frc.DashboardManagers.NetworkTablesUtil;
import com.btwrobotics.WhatTime.frc.MotorManagers.MotorWrapper;
import com.ctre.phoenix6.hardware.Pigeon2;
import java.lang.Math;
import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.subsystems.motor.MotorSubsystem;

public class ShooterSubsystem extends SubsystemBase {

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

    public double positionThreshold = 0.01; // Threshold for position manager

    public double hubHeight = 2;
    public double shooterHeight = 1;



    // TODO: create commands and set motors

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
