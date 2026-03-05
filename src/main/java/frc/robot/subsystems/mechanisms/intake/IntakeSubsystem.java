package frc.robot.subsystems.mechanisms.intake;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.MotorWrapper;
import com.btwrobotics.WhatTime.frc.MotorManagers.PositionManager;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public MotorWrapper angleMotor = new MotorWrapper(
        new TalonFX(9), false
    );

    @Override
    public void periodic() {
        Logger.recordOutput("IntakeSubsystem/Angle", angleMotor.getPosition());
    }

    public MotorWrapper intakeWheelsMotor = new MotorWrapper(
        new TalonFX(10), false
    );

    private PositionManager intakePositionManager = new PositionManager(
        IntakeConstants.minValue, 
        IntakeConstants.maxValue, 
        List.of(angleMotor), 
        0.2, // Motor Speed
        0.0, // Hold Speed
        IntakeConstants.threshold,
        0.1, // Min Speed
        0.1, // Kim Possible (kP)
        () -> angleMotor.getPosition() // Use motor encoder for position
    );

    public void setPosition(double targetPosition) {
        Logger.recordOutput("IntakeSubsystem/SetPosition", targetPosition);
        intakePositionManager.setTarget(targetPosition);
    }

    private double intakeSpeed = 0.2;

    public void setIntake(IntakeStates intakeState) {
        Logger.recordOutput("IntakeSubsystem/State", intakeState.toString());
        switch (intakeState) {
            case INTAKE_IN:
                intakeWheelsMotor.set(intakeSpeed);
                break;
            case INTAKE_OUT:
                intakeWheelsMotor.set(-intakeSpeed);
                break;
            case OFF:
                intakeWheelsMotor.set(0);
                break;
            default:
                break;
        }
    }
}
