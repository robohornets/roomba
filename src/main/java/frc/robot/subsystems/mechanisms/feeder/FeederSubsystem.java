package frc.robot.subsystems.mechanisms.feeder;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    // MARK: Feeder Bed
    public Motor feederBedMotor = new Motor(15).setFree(true);

    // MARK: Feeder Feeder
    public Motor feederFeederMotor = new Motor(16).setFree(true);

    // MARK: Shooter Feeder
    public Motor shooterFeederMotor = new Motor(12).setFree(true);

    // MARK: Periodic Loop
    @Override
    public void periodic() {
    }

    public Command runAllFeedMotors() {
        return Commands.runEnd(
            () -> {
                feederBedMotor.drive();
                feederFeederMotor.drive();
                shooterFeederMotor.drive();
            },
            () -> {
                feederBedMotor.drive(0);
                feederFeederMotor.drive(0);
                shooterFeederMotor.drive(0);
            }
        );
    }

    public void logMotorValues() {
        Logger.recordOutput("FeederSubsystem/MotorConnections/FeederBedConnected", feederBedMotor.getMotor().isConnected());
        Logger.recordOutput("FeederSubsystem/MotorConnections/FeederFeederConnected", feederFeederMotor.getMotor().isConnected());
        Logger.recordOutput("FeederSubsystem/MotorConnections/ShooterFeederConnected", shooterFeederMotor.getMotor().isConnected());
    }
}
