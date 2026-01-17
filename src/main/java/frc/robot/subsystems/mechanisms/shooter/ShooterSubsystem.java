package frc.robot.subsystems.mechanisms.shooter;

import java.util.List;

import com.btwrobotics.WhatTime.frc.MotorManagers.MotorWrapper;
import com.btwrobotics.WhatTime.frc.PositionManager;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
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



    public double getShooterMotorPitchDeg(){
        return shooterPitchMotor.getPosition() * 360;
    }
    public double getShooterPitchDeg(){
        // MARK: Should be roll?
        return shooterPigeon.getRoll().getValueAsDouble();
    }
    public double[] getDistanceToHub(){
        // MARK: still in progress
        // meters
        double[] hubPosition = new double[]{12.5, 4.5};
        
        double[] robotPosition = NetworkTableInstance.getDefault().getTable("CustomDashboard").getEntry("Pose").getDoubleArray(new double[]{0.0,0.0,0.0});

        // double degrees = motorSubsystem.rotationsToFace(new double[]{hubPosition[0] - robotPosition[0], hubPosition[1] - robotPosition[1]});

    
        return new double[]{0,0};
    }

    // TODO: calculate hub position using limelight (relative to robot) (x,y: front right of robot are positive)

}
