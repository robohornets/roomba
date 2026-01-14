package frc.robot.subsystems.mechanisms.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.Pigeon2;
import java.lang.Math;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.subsystems.motor.MotorSubsystem;

public class ShooterSubsystem extends SubsystemBase {

    MotorSubsystem motorSubsystem = new MotorSubsystem();

    public final TalonFX shooterMotor = new TalonFX(10);
    public final TalonFX shooterPitchMotor = new TalonFX(9);

    public final Pigeon2 shooterPigeon = new Pigeon2(34);


    // TODO: Calculate values

    public double angleSpeed = 0.1;
    public double angleHoldSpeed = 0.02;


    // TODO: create commands and set motors



    public double getShooterMotorPitch(){
        return shooterPitchMotor.getPosition().getValueAsDouble();
    }
    public double getShooterPitch(){
        // MARK: Should be roll?
        return shooterPigeon.getRoll(false).getValueAsDouble();
    }

    // TODO: calculate hub position using limelight (relative to robot) (x,y: front right of robot are positive)

}
