package frc.robot.subsystems.mechanisms.intake;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
    // MARK: Intake Angle
    public Motor angleMotor = new Motor(9, "Mechanisms");

    // MARK: Intake Wheels
    public Motor intakeWheelsMotor = new Motor(10, "Mechanisms")
        .setFree(true)
        .setMotorSpeed(0.2);

    // MARK: Angle Control State
    private double angleTarget = Double.NaN;
    private static final double ANGLE_SPEED = 0.3;

    public IntakeSubsystem() {
        angleMotor.toggleEnabled(true);
        intakeWheelsMotor.toggleEnabled(true);

        angleMotor.setDefaultCommand(Commands.run(() -> {}, angleMotor));
        intakeWheelsMotor.setDefaultCommand(Commands.run(() -> {}, intakeWheelsMotor));

        angleMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        intakeWheelsMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
    }

    // MARK: Periodic Loop
    @Override
    public void periodic() {
        runAngleControl();

        Logger.recordOutput("IntakeSubsystem/Angle", angleMotor.getMotor().getPosition().refresh().getValueAsDouble());
        Logger.recordOutput("IntakeSubsystem/AngleTarget", Double.isNaN(angleTarget) ? -1.0 : angleTarget);
        Logger.recordOutput("IntakeSubsystem/AngleSpeed", angleMotor.getMotor().get());
    }

    // MARK: Angle Control
    private void runAngleControl() {
        if (Double.isNaN(angleTarget)) {
            angleMotor.getMotor().set(0.0);
            return;
        }

        double currentPos = angleMotor.getMotor().getPosition().refresh().getValueAsDouble();
        double error = angleTarget - currentPos;

        if (Math.abs(error) <= IntakeConstants.threshold) {
            angleMotor.getMotor().set(0.0);
        } else {
            angleMotor.getMotor().set(Math.copySign(ANGLE_SPEED, error));
        }
    }

    public void setPosition(double targetPosition) {
        Logger.recordOutput("IntakeSubsystem/SetPosition", targetPosition);
        angleTarget = targetPosition;
    }

    private double intakeSpeed = 0.2;

    public void setIntake(IntakeStates intakeState) {
        Logger.recordOutput("IntakeSubsystem/State", intakeState.toString());
        switch (intakeState) {
            case INTAKE_IN:
                intakeWheelsMotor.getMotor().set(intakeSpeed);
                break;
            case INTAKE_OUT:
                intakeWheelsMotor.getMotor().set(-intakeSpeed);
                break;
            case OFF:
                intakeWheelsMotor.getMotor().set(0.0);
                break;
            default:
                break;
        }
    }
}
