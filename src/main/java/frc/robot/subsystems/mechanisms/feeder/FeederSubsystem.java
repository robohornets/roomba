package frc.robot.subsystems.mechanisms.feeder;

import org.littletonrobotics.junction.Logger;
import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class FeederSubsystem extends SubsystemBase {
    // MARK: Feeder Bed
    public Motor feederBedMotor = new Motor(15, "Mechanisms", true).setFree(true).setMotorSpeed(FeederConstants.FEEDER_BED_SPEED);

    // MARK: Feeder Feeder
    public Motor feederFeederMotor = new Motor(16, "Mechanisms").setFree(true).setMotorSpeed(FeederConstants.FEEDER_FEEDER_SPEED);

    // MARK: Shooter Feeder
    public Motor shooterFeederMotor = new Motor(12, "Mechanisms").setFree(true).setMotorSpeed(FeederConstants.SHOOTER_FEEDER_SPEED);

    // MARK: Feeder State
    private FeederState feederState = FeederState.OFF;

    // MARK: Constructor
    public FeederSubsystem() {
        feederBedMotor.toggleEnabled(true);
        feederFeederMotor.toggleEnabled(true);
        shooterFeederMotor.toggleEnabled(true);

        feederBedMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        feederFeederMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        shooterFeederMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);

        feederBedMotor.setDefaultCommand(Commands.run(() -> {}, feederBedMotor));
        feederFeederMotor.setDefaultCommand(Commands.run(() -> {}, feederFeederMotor));
        shooterFeederMotor.setDefaultCommand(Commands.run(() -> {}, shooterFeederMotor));
    }

    // MARK: Periodic Loop
    @Override
    public void periodic() {
        logValues();

        switch (feederState) {
            case ALL_FEEDER_IN:
                runSpecifiedMotors(true, false, true, false, true, true);
                break;

            case ALL_FEEDER_OUT:
                runSpecifiedMotors(true, true, true, true, true, false);
                break;

            case SHOOTER_FEED_IN:
                runSpecifiedMotors(true, false, true, false, false, false);
                break;

            case SHOOTER_FEED_OUT:
                runSpecifiedMotors(true, true, true, true, false, false);
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
        boolean runFeederFeeder,
        boolean feederFeederInverted,
        boolean runShooterFeeder,
        boolean shooterFeederInverted,
        boolean runBed,
        boolean bedInverted
    ) {
        double feederSpeed = runFeederFeeder ? (feederFeederInverted ? -FeederConstants.FEEDER_FEEDER_SPEED : FeederConstants.FEEDER_FEEDER_SPEED) : 0.0;
        double shooterSpeed = runShooterFeeder ? (shooterFeederInverted ? -FeederConstants.SHOOTER_FEEDER_SPEED : FeederConstants.SHOOTER_FEEDER_SPEED) : 0.0;
        double bedSpeed = runBed ? (bedInverted ? -FeederConstants.FEEDER_BED_SPEED : FeederConstants.FEEDER_BED_SPEED) : 0.0;

        feederFeederMotor.getMotor().set(feederSpeed);
        shooterFeederMotor.getMotor().set(shooterSpeed);
        feederBedMotor.getMotor().set(bedSpeed);

        Logger.recordOutput("FeederSubsystem/FeederFeederSpeed", feederSpeed);
        Logger.recordOutput("FeederSubsystem/ShooterFeederSpeed", shooterSpeed);
        Logger.recordOutput("FeederSubsystem/BedSpeed", bedSpeed);
        Logger.recordOutput("FeederSubsystem/FeederState", feederState.toString());
    }

    public void setFeederState(FeederState feederState) {
        this.feederState = feederState;
    }

    // MARK: Logging
    public void logValues() {
        Logger.recordOutput("FeederSubsystem/MotorConnections/FeederBedConnected", feederBedMotor.getMotor().isConnected());
        Logger.recordOutput("FeederSubsystem/MotorConnections/FeederFeederConnected", feederFeederMotor.getMotor().isConnected());
        Logger.recordOutput("FeederSubsystem/MotorConnections/ShooterFeederConnected", shooterFeederMotor.getMotor().isConnected());

        Logger.recordOutput("FeederSubsystem/FeederState", feederState.toString());
    }
}
