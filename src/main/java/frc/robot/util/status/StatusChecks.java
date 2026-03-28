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
    public boolean visionChecksPassed = false;

    public String checkStatusIndicator = StatusCheckConstants.STATUS_BAD_HEX;

    public void runAllStatusChecks() {
        allChecksPassed = drivetrainStatusChecks() && 
            shooterStatusChecks() && 
            intakeStatusChecks() && 
            feederStatusChecks();
        visionChecksPassed = visionStatusChecks();

        // If vision and mechanisms pass, display green
        if (allChecksPassed && visionChecksPassed) {
            checkStatusIndicator = StatusCheckConstants.STATUS_GOOD_HEX;
        }
        // If only mechanism subsystems pass, display yellow
        else if (allChecksPassed) {
            checkStatusIndicator = StatusCheckConstants.STATUS_NEUTRAL_HEX;
        }
        // If mechanisms do not pass, display red
        else {
            checkStatusIndicator = StatusCheckConstants.STATUS_BAD_HEX;
        }

        Logger.recordOutput("StatusChecks/StatusCheckIndicator", checkStatusIndicator);
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

    private boolean visionStatusChecks() {
        boolean visionConnected = drivetrain.questNavSubsystem.questIsConnected() &&
            drivetrain.limelightSubsystem.limelightIsConnected();
        
        return visionConnected;
    }

    private boolean genericMotorCheck(Motor motor) {
        return motor.getMotor().isConnected();
    }
}
