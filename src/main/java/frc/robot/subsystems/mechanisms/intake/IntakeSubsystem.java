package frc.robot.subsystems.mechanisms.intake;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
    // MARK: Intake Angle
    // public Motor angleMotor = new Motor(9, "Mechanisms");
    public TalonFX angleMotor = new TalonFX(9, "Mechanisms");

    // MARK: Intake Wheels
    public Motor intakeWheelsMotor = new Motor(10, "Mechanisms");

    // MARK: Angle Control State
    private double angleTarget = Double.NaN;
    private static final double ANGLE_DOWN_SPEED = 0.2;
    private static final double ANGLE_UP_SPEED = 0.2;

    public IntakeSubsystem() {
        // angleMotor.toggleEnabled(true);
        intakeWheelsMotor.toggleEnabled(true);

        // angleMotor.setDefaultCommand(Commands.run(() -> {}, angleMotor));
        intakeWheelsMotor.setDefaultCommand(Commands.run(() -> {}, intakeWheelsMotor));

        angleMotor.getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        intakeWheelsMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
    }

    // MARK: Intake Wheel State
    private IntakeStates intakeState = IntakeStates.OFF;
    private static final double INTAKE_SPEED = 0.2;

    // MARK: Periodic Loop
    @Override
    public void periodic() {
        runAngleControl();
        runIntakeWheels();

        Logger.recordOutput("IntakeSubsystem/Angle", angleMotor.getPosition().refresh().getValueAsDouble());
        Logger.recordOutput("IntakeSubsystem/AngleTarget", Double.isNaN(angleTarget) ? -1.0 : angleTarget);
        Logger.recordOutput("IntakeSubsystem/AngleSpeed", angleMotor.get());
        Logger.recordOutput("IntakeSubsystem/WheelState", intakeState.toString());
    }

    // MARK: Angle Control
    private void runAngleControl() {
        if (Double.isNaN(angleTarget)) {
            angleMotor.set(0.0);
            return;
        }

        double currentPos = angleMotor.getPosition().refresh().getValueAsDouble();
        double error = angleTarget - currentPos;

        if (Math.abs(error) <= IntakeConstants.threshold) {
            angleMotor.set(0.0);
        } else if (error < 0) {
            angleMotor.set(Math.copySign(ANGLE_DOWN_SPEED, error));
        }
        else {
            angleMotor.set(Math.copySign(ANGLE_UP_SPEED, error));
        }
    }

    // MARK: Intake Wheels
    private void runIntakeWheels() {
        switch (intakeState) {
            case INTAKE_IN:
                intakeWheelsMotor.getMotor().set(INTAKE_SPEED);
                break;
            case INTAKE_OUT:
                intakeWheelsMotor.getMotor().set(-INTAKE_SPEED);
                break;
            case OFF:
                intakeWheelsMotor.getMotor().set(0.0);
                break;
            default:
                intakeWheelsMotor.getMotor().set(0.0);
                break;
        }
    }

    public void setPosition(double targetPosition) {
        Logger.recordOutput("IntakeSubsystem/SetPosition", targetPosition);
        angleTarget = targetPosition;
    }

    public void setIntake(IntakeStates intakeState) {
        this.intakeState = intakeState;
    }
}
