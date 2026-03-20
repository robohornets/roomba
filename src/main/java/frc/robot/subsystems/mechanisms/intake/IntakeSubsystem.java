package frc.robot.subsystems.mechanisms.intake;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
    // MARK: Intake Angle
    public Motor angleMotor = new Motor(9, "Mechanisms")
        .setFree(false)
        .setMotorSpeed(1.0)
        .setMinValue(0.0)
        .setMaxValue(2.34)
        .setThreshold(0.2);
    // public TalonFX angleMotor = new TalonFX(9, "Mechanisms");


    // MARK: Intake Wheels
    public Motor intakeWheelsMotor = new Motor(10, "Mechanisms");

    public IntakeSubsystem() {
        angleMotor.toggleEnabled(true);
        intakeWheelsMotor.toggleEnabled(true);

        intakeWheelsMotor.setDefaultCommand(Commands.run(() -> {}, intakeWheelsMotor));

        angleMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        intakeWheelsMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
    }

    // MARK: Intake Wheel State
    private IntakeStates intakeState = IntakeStates.OFF;
    public IntakeStates lastIntakeState = IntakeStates.OFF;

    // MARK: Periodic Loop
    @Override
    public void periodic() {
        // runAngleControl();
        runIntakeWheels();

        logValues();
    }

    // MARK: Angle Control
    // private void runAngleControl() {
    //     if (Double.isNaN(angleTarget)) {
    //         angleMotor.set(0.0);
    //         return;
    //     }

    //     double currentPos = angleMotor.getMotor().getPosition().refresh().getValueAsDouble();
    //     double error = angleTarget - currentPos;

    //     if (Math.abs(error) <= IntakeConstants.INTAKE_THRESHOLD) {
    //         angleMotor.set(0.0);
    //     } else if (error < 0) {
    //         angleMotor.set(Math.copySign(IntakeConstants.INTAKE_DOWN_SPEED, error));
    //     }
    //     else {
    //         angleMotor.set(Math.copySign(IntakeConstants.INTAKE_UP_SPEED, error));
    //     }
    // }

    // MARK: Intake Wheels
    private void runIntakeWheels() {
        switch (intakeState) {
            case INTAKE_IN:
                intakeWheelsMotor.drive(IntakeConstants.INTAKE_WHEELS_SPEED);
                break;
            case INTAKE_OUT:
                intakeWheelsMotor.drive(-IntakeConstants.INTAKE_WHEELS_SPEED);
                break;
            case OFF:
                intakeWheelsMotor.drive(0.0);
                break;
            default:
                intakeWheelsMotor.drive(0.0);
                break;
        }
    }

    public void setPosition(double targetPosition) {
        Logger.recordOutput("IntakeSubsystem/SetPosition", targetPosition);
        angleMotor.goTo(targetPosition);
    }

    public IntakeStates getIntakeState() {
        return intakeState;
    }

    public void setIntake(IntakeStates intakeState) {
        if (!intakeState.equals(this.intakeState)) {
            lastIntakeState = this.intakeState;
        }

        this.intakeState = intakeState;
    }

    // MARK: Logging
    private void logValues() {
        Logger.recordOutput("IntakeSubsystem/Angle", angleMotor.getMotor().getPosition().refresh().getValueAsDouble());
        // Logger.recordOutput("IntakeSubsystem/AngleTarget", Double.isNaN(angleTarget) ? -1.0 : angleTarget);
        Logger.recordOutput("IntakeSubsystem/AngleSpeed", angleMotor.getMotor().get());
        Logger.recordOutput("IntakeSubsystem/WheelState", intakeState.toString());
        Logger.recordOutput("IntakeSubsystem/Current/AngleMotor", angleMotor.getMotor().getStatorCurrent().getValueAsDouble());
    }
}
