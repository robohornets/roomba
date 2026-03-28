package frc.robot.util.status;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.feeder.FeederSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;

public class StatusChecks {
    private Drive drivetrain;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private FeederSubsystem feederSubsystem;

    public StatusChecks(
        Drive drivetrain,
        ShooterSubsystem shooterSubsystem,
        IntakeSubsystem intakeSubsystem,
        FeederSubsystem feederSubsystem
    ) {
        this.drivetrain = drivetrain;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.feederSubsystem = feederSubsystem;
    }

    public boolean allChecksPassed = false;

    public void runAllStatusChecks() {
        allChecksPassed = drivetrainStatusChecks() && shooterStatusChecks() && intakeStatusChecks() && feederStatusChecks();

        Logger.recordOutput("StatusChecks/AllChecksPassed", allChecksPassed);
    }

    private boolean drivetrainStatusChecks() {
        for (int i = 0; i <= 3; i++) {
            if (
                !drivetrain.drivetrain.getModule(i).getSteerMotor().isConnected() &&
                !drivetrain.drivetrain.getModule(i).getDriveMotor().isConnected() &&
                !drivetrain.drivetrain.getModule(i).getEncoder().isConnected()
            ) {
                return false;
            }
        }
        return true;
    }

    private boolean shooterStatusChecks() {
        boolean motorsConnected = genericMotorCheck(shooterSubsystem.leftShooterMotor) && 
            genericMotorCheck(shooterSubsystem.rightShooterMotor) &&
            genericMotorCheck(shooterSubsystem.shooterPitchMotor);
        
        return motorsConnected;
    }

    private boolean intakeStatusChecks() {
        boolean motorsConnected = genericMotorCheck(intakeSubsystem.angleMotor) && 
            genericMotorCheck(intakeSubsystem.intakeWheelsMotor);

        return motorsConnected;
    }

    private boolean feederStatusChecks() {
        boolean motorsConnected = genericMotorCheck(feederSubsystem.feederBedMotor) && 
            genericMotorCheck(feederSubsystem.feederFeederMotor) &&
            genericMotorCheck(feederSubsystem.shooterFeederMotor);

        return motorsConnected;
    }

    private boolean genericMotorCheck(Motor motor) {
        return motor.getMotor().isConnected();
    }
}
