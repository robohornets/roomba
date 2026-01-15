package frc.robot.subsystems.mechanisms.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.btwrobotics.WhatTime.frc.FlywheelPair;
import com.btwrobotics.WhatTime.frc.MotorManagers.MotorWrapper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    // TODO: Recalculate all these values
    public final double climberUpDownSpeed = 0.5;
    public final double climberUpDownSpeedSlow = 0.1;

    public static final double minHeight = 0.0; 
    public static final double maxHeight = 65.0;

    
    
    
    // TODO: VERY IMPORTANT!!!! Put the correct device ids
    public final MotorWrapper climberLeft = new MotorWrapper(
        new TalonFX(12),
        true
    );
    public final MotorWrapper climberRight = new MotorWrapper(
        new TalonFX(13),
        false
    );
    public final FlywheelPair climberPair = new FlywheelPair(climberLeft, climberRight, climberUpDownSpeed);



    public double elevatorEncoderOffset = 0.0;

    public void resetClimberEncoder() {
        elevatorEncoderOffset = climberLeft.getPosition();
    }

    public double getClimberHeight() {
        return climberLeft.getPosition();
    }


    // TODO: Caedmon mentioned something about this being implemented differently, needs to be done (like we dont have to make the motors rotate opposite)
    public Command climberUp() {
        return Commands.run(
            () -> {
                if (Math.abs(climberLeft.getPosition()) <= 65.0) {
                    climberPair.runForward();
                    CommandScheduler.getInstance().cancelAll();
                } else {
                    climberPair.runForward(0.015);
                }
            }
        );
    }

    public Command climberDown() {
        return Commands.run(
            () -> {
                if (Math.abs(climberLeft.getPosition()) >= 5.0) {
                    climberPair.runBackward();
                } else {
                    climberPair.runForward(0.015);
                }
            }
        );
    }

    public Command climberUpManual() {
        return Commands.run(
            () -> {
                climberPair.runForward();
            }
        );
    }

    public Command climberDownManual() {
        return Commands.run(
            () -> {
                climberPair.runBackward();
            }
        );
    }

    public Command climberUpSlow() {
        return Commands.run(
            () -> {
                if (Math.abs(climberLeft.getPosition()) <= 65.0) {
                    climberPair.runForward(climberUpDownSpeedSlow);
                } else {
                    climberPair.runForward(0.015);
                }
            }
        );
    }

    public Command climberDownSlow() {
        return Commands.run(
            () -> {
                if (Math.abs(climberLeft.getPosition()) >= 5.0) {
                    climberPair.runBackward(-climberUpDownSpeedSlow);
                } else {
                    climberPair.runForward(0.015);
                }
            }
        );
    }

    public Command stopClimber() {
        return Commands.run(
            () -> {
                climberPair.runForward(0.015);
            }
        );
    }
}
