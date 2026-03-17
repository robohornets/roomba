package frc.robot.subsystems.mechanisms.intake;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
    // MARK: Intake Angle
    public Motor angleMotor = new Motor(9, "Mechanisms")
        .setFree(false)
        .setMinValue(IntakeConstants.minValue)
        .setMaxValue(IntakeConstants.maxValue)
        .setMotorSpeed(0.8)
        .setHoldSpeed(0.0)
        .setThreshold(IntakeConstants.threshold)
        .setMinSpeed(0.3)
        .setPG(0.8);

    // MARK: Intake Wheels
    public Motor intakeWheelsMotor = new Motor(10, "Mechanisms")
        .setFree(true)
        .setMotorSpeed(0.2);

    public IntakeSubsystem() {
        angleMotor.setPositionSupplier(() -> angleMotor.getMotor().getPosition().refresh().getValueAsDouble());

        angleMotor.toggleEnabled(true);
        intakeWheelsMotor.toggleEnabled(true);
        
        angleMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        intakeWheelsMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
    }

    // MARK: Periodic Loop
    @Override
    public void periodic() {
        Logger.recordOutput("IntakeSubsystem/Angle", angleMotor.getCurrentValue());
        Logger.recordOutput("IntakeSubsystem/IntakeAngleSpeed", angleMotor.getMotor().get());
    }

    public void setPosition(double targetPosition) {
        Logger.recordOutput("IntakeSubsystem/SetPosition", targetPosition);
        angleMotor.goTo(targetPosition);
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
