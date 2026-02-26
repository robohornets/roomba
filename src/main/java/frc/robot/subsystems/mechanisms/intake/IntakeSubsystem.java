package frc.robot.subsystems.mechanisms.intake;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.MotorWrapper;
import com.btwrobotics.WhatTime.frc.MotorManagers.PositionManager;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final double minValue = 0.0;
    private final double maxValue = 1.0;
    private final double threshold = 0.2;

    private List<MotorWrapper> angleMotors = List.of(
        new MotorWrapper(
            new TalonFX(9), false
        )
    );

    @Override
    public void periodic() {
        Logger.recordOutput("IntakeSubsystem/Angle", angleMotors.get(0).getPosition());
    }

    private MotorWrapper intakeWheelsMotor = new MotorWrapper(
        new TalonFX(10), false
    );

    // private CANcoder angleEncoder = new CANcoder(35);

    private PositionManager intakePositionManager = new PositionManager(
        minValue, 
        maxValue, 
        angleMotors, 
        0.2, 
        0.0, 
        threshold,
        0.02, 
        () -> intakeWheelsMotor.getPosition() // Use motor encoder for position
    );

    public void setPosition(double targetPosition) {
        Logger.recordOutput("IntakeSubsystem/SetPosition", targetPosition);
        intakePositionManager.setTarget(targetPosition);
    }

    private double intakeSpeed = 0.5;

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
