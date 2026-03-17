package frc.robot.subsystems.mechanisms.feeder;

import org.littletonrobotics.junction.Logger;
import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;

import edu.wpi.first.wpilibj.Timer;
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

    // MARK: Bed Agitation Timer
    private final Timer bedAgitationTimer = new Timer();
    private static final double BED_FORWARD_SECONDS = 10.0;
    private static final double BED_REVERSE_SECONDS = 2.0;

    // MARK: Constructor
    public FeederSubsystem() {
        feederBedMotor.toggleEnabled(true);
        feederFeederMotor.toggleEnabled(true);
        shooterFeederMotor.toggleEnabled(true);

        feederBedMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        feederFeederMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        shooterFeederMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);

        bedAgitationTimer.start();
    }

    // MARK: Periodic Loop
    @Override
    public void periodic() {
        runBedAgitation();

        switch (feederState) {
            case ALL_FEEDER_IN:
                runSpecifiedMotors(true, false, true, false);
                break;

            case ALL_FEEDER_OUT:
                runSpecifiedMotors(true, true, true, true);
                break;

            case SHOOTER_FEED_IN:
                runSpecifiedMotors(true, false, true, false);
                break;

            case SHOOTER_FEED_OUT:
                runSpecifiedMotors(true, true, true, true);
                break;

            case OFF:
                runSpecifiedMotors(false, false, false, false);
                break;

            default:
                runSpecifiedMotors(false, false, false, false);
                break;
        }
    }

    // MARK: Bed Agitation
    // Drives forward for 10 seconds, reverses for 2 seconds, repeats.
    private void runBedAgitation() {
        double t = bedAgitationTimer.get() % (BED_FORWARD_SECONDS + BED_REVERSE_SECONDS);
        if (t < BED_FORWARD_SECONDS) {
            feederBedMotor.drive();
        } else {
            feederBedMotor.drive(true);
        }

        Logger.recordOutput("FeederSubsystem/BedAgitationTime", t);
    }

    private void runSpecifiedMotors(
        boolean runFeederFeeder,
        boolean feederFeederInverted,
        boolean runShooterFeeder,
        boolean shooterFeederInverted
    ) {
        double feederSpeed = runFeederFeeder ? (feederFeederInverted ? -FeederConstants.FEEDER_FEEDER_SPEED : FeederConstants.FEEDER_FEEDER_SPEED) : 0.0;
        double shooterSpeed = runShooterFeeder ? (shooterFeederInverted ? -FeederConstants.SHOOTER_FEEDER_SPEED : FeederConstants.SHOOTER_FEEDER_SPEED) : 0.0;

        feederFeederMotor.getMotor().set(feederSpeed);
        shooterFeederMotor.getMotor().set(shooterSpeed);

        Logger.recordOutput("FeederSubsystem/FeederFeederSpeed", feederSpeed);
        Logger.recordOutput("FeederSubsystem/ShooterFeederSpeed", shooterSpeed);
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
