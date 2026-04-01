package frc.robot.subsystems.mechanisms.intake;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;
import com.btwrobotics.WhatTime.frc.MotorManagers.MotorGroup;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
    // MARK: Intake Angle
    public Motor angleMotor = new Motor(9, "Mechanisms")
        .setFree(false)
        .setMotorSpeed(0.5)
        .setMinValue(IntakeConstants.INTAKE_MIN_VALUE)
        .setMaxValue(IntakeConstants.INTAKE_MAX_VALUE)
        .setPG(0.5)
        .setThreshold(IntakeConstants.INTAKE_THRESHOLD);
    // public TalonFX angleMotor = new TalonFX(9, "Mechanisms");


    // MARK: Intake Wheels
    public Motor intakeWheelLeft = new Motor(10, "Mechanisms");
    public Motor intakeWheelRight = new Motor(10, "Mechanisms", true);

    public MotorGroup intakeWheelsMotor = new MotorGroup(Arrays.asList(intakeWheelLeft, intakeWheelRight));


    public IntakeSubsystem() {
        angleMotor.toggleEnabled(true);
        intakeWheelsMotor.toggleEnabled(true);

        angleMotor.setNeutralMode(NeutralModeValue.Brake);

        intakeWheelsMotor.setDefaultCommand(Commands.run(() -> {}, intakeWheelsMotor));

        angleMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        intakeWheelLeft.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        intakeWheelRight.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
    
    }

    // MARK: Intake Wheel State
    private IntakeStates intakeState = IntakeStates.OFF;
    public IntakeStates lastIntakeState = IntakeStates.OFF;
    
    // MARK: Intake Angle State
    public double anglePosition = 0.0;
    private double angleTarget = Double.NaN;
    public double angleMinimum = IntakeConstants.INTAKE_MIN_VALUE;
    // public boolean forceIntakeDown = false;


    // MARK: Periodic Loop
    @Override
    public void periodic() {
        // runAngleControl();

        logValues();
        logMotors();
    }

    // MARK: Angle Control
    // private void runAngleControl() {
    //     // Force the intake down with constant motor speed
    //     // if (forceIntakeDown) {
    //     //     // angleMotor.getMotor().set(IntakeConstants.INTAKE_DOWN_FORCE_SPEED);
    //     //     return;
    //     // }

    //     if (Double.isNaN(angleTarget) || angleAtTarget) {
    //         angleMotor.set(0.0);
    //         return;
    //     }

    //     double currentPos = angleMotor.getMotor().getPosition().refresh().getValueAsDouble();
    //     double error = angleTarget - currentPos;

    //     if (Math.abs(error) <= IntakeConstants.INTAKE_THRESHOLD) {
    //         angleMotor.set(0.0);
    //         angleAtTarget = true;
    //     } else if (error < 0) {
    //         angleMotor.set(-IntakeConstants.INTAKE_DOWN_SPEED);
    //     } else {
    //         angleMotor.set(IntakeConstants.INTAKE_UP_SPEED);
    //     }
    // }

    public void setPosition(double targetPosition) {
        angleMotor.goTo(targetPosition);
        angleTarget = targetPosition;
    }

    public IntakeStates getIntakeState() {
        return intakeState;
    }

    public void setIntake(IntakeStates intakeState) {
        if (!intakeState.equals(this.intakeState)) {
            lastIntakeState = this.intakeState;
        }

        this.intakeState = intakeState;

        switch (intakeState) {
            case INTAKE_IN:
                intakeWheelsMotor.drive(IntakeConstants.INTAKE_WHEELS_SPEED);
                break;
            case INTAKE_OUT:
                intakeWheelsMotor.drive(-IntakeConstants.INTAKE_WHEELS_SPEED / 2);
                break;
            case OFF:
                intakeWheelsMotor.drive(0.0);
                break;
            default:
                intakeWheelsMotor.drive(0.0);
                break;
        }

        // forceIntakeDown = intakeState.equals(IntakeStates.INTAKE_IN);
    }

    // MARK: Logging
    private void logValues() {
        Logger.recordOutput("IntakeSubsystem/Angle", angleMotor.getMotor().getPosition().refresh().getValueAsDouble());
        Logger.recordOutput("IntakeSubsystem/AngleTarget", angleTarget);
        Logger.recordOutput("IntakeSubsystem/AngleSpeed", angleMotor.getMotor().get());
        Logger.recordOutput("IntakeSubsystem/WheelState", intakeState.toString());
    }

    // MARK: Log Motors
    private void logMotors() {
        // Log connection status
        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/MotorConnections/AngleMotor", 
            angleMotor.getMotor().isConnected()
        );

        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/MotorConnections/IntakeLeftMotor", 
            intakeWheelLeft.getMotor().isConnected()
        );

        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/MotorConnections/IntakeRightMotor", 
            intakeWheelRight.getMotor().isConnected()
        );

        // Log current readings to AdvantageKit
        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/Current/Stator/AngleMotor", 
            angleMotor.getMotor().getStatorCurrent().getValueAsDouble()
        );
        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/Current/Supply/AngleMotor", 
            angleMotor.getMotor().getSupplyCurrent().getValueAsDouble()
        );

        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/Current/Stator/IntakeLeftMotor", 
            intakeWheelLeft.getMotor().getStatorCurrent().getValueAsDouble()
        );
        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/Current/Supply/IntakeLeftMotor", 
            intakeWheelLeft.getMotor().getSupplyCurrent().getValueAsDouble()
        );

        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/Current/Stator/IntakeRightMotor", 
            intakeWheelRight.getMotor().getStatorCurrent().getValueAsDouble()
        );
        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/Current/Supply/IntakeRightMotor", 
            intakeWheelRight.getMotor().getSupplyCurrent().getValueAsDouble()
        );

        // Log voltage readings to AdvantageKit
        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/Voltage/Output/FeederBedMotor", 
            angleMotor.getMotor().getMotorVoltage().getValueAsDouble()
        );
        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/Current/Supply/FeederBedMotor", 
            angleMotor.getMotor().getSupplyVoltage().getValueAsDouble()
        );

        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/Voltage/Output/IntakeLeftMotor", 
            intakeWheelLeft.getMotor().getMotorVoltage().getValueAsDouble()
        );
        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/Current/Supply/IntakeLeftMotor", 
            intakeWheelLeft.getMotor().getSupplyVoltage().getValueAsDouble()
        );

        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/Voltage/Output/IntakeRightMotor", 
            intakeWheelRight.getMotor().getMotorVoltage().getValueAsDouble()
        );
        Logger.recordOutput(
            "MotorStatus/IntakeSubsystem/Current/Supply/IntakeRightMotor", 
            intakeWheelRight.getMotor().getSupplyVoltage().getValueAsDouble()
        );
    }
}
