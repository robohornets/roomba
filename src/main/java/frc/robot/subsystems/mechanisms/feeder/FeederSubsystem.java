package frc.robot.subsystems.mechanisms.feeder;

import org.littletonrobotics.junction.Logger;
import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class FeederSubsystem extends SubsystemBase {
    // MARK: Feeder Bed
    public Motor feederBedMotor = new Motor(15, "Mechanisms").setFree(true).setMotorSpeed(FeederConstants.FEEDER_BED_SPEED);

    // MARK: Feeder Feeder
    public Motor feederFeederMotor = new Motor(16, "Mechanisms").setFree(true).setMotorSpeed(FeederConstants.FEEDER_FEEDER_SPEED);

    // MARK: Shooter Feeder
    public Motor shooterFeederMotor = new Motor(12, "Mechanisms").setFree(true).setMotorSpeed(FeederConstants.SHOOTER_FEEDER_SPEED);

    // MARK: Feeder State
    private FeederState feederState = FeederState.OFF;

    // Applys current limits to motors to reduce chance of brownout
    // MARK: Constructor
    public FeederSubsystem() {
        feederBedMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        feederFeederMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        shooterFeederMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
    }

    // MARK: Periodic Loop
    @Override
    public void periodic() {
        switch (feederState) {
            case ALL_FEEDER_IN:
                runSpecifiedMotors(true, false, true, false, true, false);
                break;
            
            case ALL_FEEDER_OUT:
                runSpecifiedMotors(true, true, true, true, true, true);
                break;

            case ROLLERS_IN:
                runSpecifiedMotors(true, false, false, false, false, false);
                break;

            case ROLLERS_OUT:
                runSpecifiedMotors(true, true, false, false, false, false);
                break;

            case SHOOTER_FEED_IN:
                runSpecifiedMotors(false, false, true, false, true, false);
                break;

            case SHOOTER_FEED_OUT:
                runSpecifiedMotors(false, false, true, true, true, true);
                break;
                
            case OFF:
                runSpecifiedMotors(false, false, false, false, false, false);
                break;
        
            default:
                runSpecifiedMotors(false, false, false, false, false, false);
                break;
        }
    }

    private void runSpecifiedMotors(
        boolean runFeederBed, 
        boolean feederBedInverted, 
        boolean runFeederFeeder, 
        boolean feederFeederInverted,
        boolean runShooterFeeder,
        boolean shooterFeederInverted
    ) {
        feederBedMotor.toggleEnabled(runFeederBed);
        feederFeederMotor.toggleEnabled(runFeederFeeder);
        shooterFeederMotor.toggleEnabled(runShooterFeeder);

        if (runFeederBed) {
            feederBedMotor.drive(feederBedInverted);
        }
        if (runFeederFeeder) {
            feederFeederMotor.drive(feederFeederInverted);
        }
        if (runShooterFeeder) {
            shooterFeederMotor.drive(shooterFeederInverted);
        }
    }

    public void setFeederState(FeederState feederState) {
        this.feederState = feederState;
    }

    public void logValues() {
        Logger.recordOutput("FeederSubsystem/MotorConnections/FeederBedConnected", feederBedMotor.getMotor().isConnected());
        Logger.recordOutput("FeederSubsystem/MotorConnections/FeederFeederConnected", feederFeederMotor.getMotor().isConnected());
        Logger.recordOutput("FeederSubsystem/MotorConnections/ShooterFeederConnected", shooterFeederMotor.getMotor().isConnected());

        Logger.recordOutput("FeederSubsystem/FeederState", feederState.toString());
    }
}
