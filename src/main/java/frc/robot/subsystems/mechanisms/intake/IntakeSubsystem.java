package frc.robot.subsystems.mechanisms.intake;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // MARK: Intake Angle
    public Motor angleMotor = new Motor(9)
        .setFree(false);

    // MARK: Intake Wheels
    public Motor intakeWheelsMotor = new Motor(10)
        .setMinValue(IntakeConstants.minValue)
        .setMaxValue(IntakeConstants.maxValue)
        .setMotorSpeed(0.2)
        .setHoldSpeed(0.0)
        .setThreshold(IntakeConstants.threshold)
        .setMinSpeed(0.1)
        .setPG(0.1)
        .setPositionSupplier(()-> angleMotor.getCurrentValue());

    // MARK: Periodic Loop
    @Override
    public void periodic() {
        Logger.recordOutput("IntakeSubsystem/Angle", angleMotor.getCurrentValue());
    }

    public void setPosition(double targetPosition) {
        Logger.recordOutput("IntakeSubsystem/SetPosition", targetPosition);
        intakeWheelsMotor.goTo(targetPosition);
    }

    private double intakeSpeed = 0.2;

    public void setIntake(IntakeStates intakeState) {
        Logger.recordOutput("IntakeSubsystem/State", intakeState.toString());
        switch (intakeState) {
            case INTAKE_IN:
                intakeWheelsMotor.drive(intakeSpeed);
                break;
            case INTAKE_OUT:
                intakeWheelsMotor.drive(-intakeSpeed);
                break;
            case OFF:
                intakeWheelsMotor.drive(0);
                break;
            default:
                break;
        }
    }
}
