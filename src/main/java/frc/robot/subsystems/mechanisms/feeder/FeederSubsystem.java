package frc.robot.subsystems.mechanisms.feeder;

import org.littletonrobotics.junction.Logger;
import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        feederBedMotor.toggleEnabled(true);
        feederFeederMotor.toggleEnabled(true);
        shooterFeederMotor.toggleEnabled(true);

        feederBedMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        feederFeederMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        shooterFeederMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);

        motorBedAgitationRoutine().schedule();
    }

    // MARK: Periodic Loop
    @Override
    public void periodic() {
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
    // Reverses the bed roller motors every 10 seconds
    private Command motorBedAgitationRoutine() {
        return Commands.repeatingSequence(
            Commands.run(
                () -> {
                    feederBedMotor.drive();
                }
            ).withTimeout(10),
            Commands.run(
                () -> {
                    feederBedMotor.drive(true);
                }
            ).withTimeout(2)
        );
    }

    private void runSpecifiedMotors(
        boolean runFeederFeeder, 
        boolean feederFeederInverted,
        boolean runShooterFeeder,
        boolean shooterFeederInverted
    ) {
        feederFeederMotor.toggleEnabled(runFeederFeeder);
        shooterFeederMotor.toggleEnabled(runShooterFeeder);

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

    // MARK: Logging
    public void logValues() {
        Logger.recordOutput("FeederSubsystem/MotorConnections/FeederBedConnected", feederBedMotor.getMotor().isConnected());
        Logger.recordOutput("FeederSubsystem/MotorConnections/FeederFeederConnected", feederFeederMotor.getMotor().isConnected());
        Logger.recordOutput("FeederSubsystem/MotorConnections/ShooterFeederConnected", shooterFeederMotor.getMotor().isConnected());

        Logger.recordOutput("FeederSubsystem/FeederState", feederState.toString());
    }
}
