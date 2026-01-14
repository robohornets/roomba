package frc.robot.subsystems.mechanisms.climber;

import com.ctre.phoenix6.hardware.TalonFX;

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
    public final TalonFX climberLeft = new TalonFX(12);
    public final TalonFX climberRight = new TalonFX(13);

    public double elevatorEncoderOffset = 0.0;

    public void resetClimberEncoder() {
        elevatorEncoderOffset = climberLeft.getPosition().getValueAsDouble();
    }

    public double getClimberHeight() {
        return climberLeft.getPosition().getValueAsDouble();
    }


    // TODO: Caedmon mentioned something about this being implemented differently, needs to be done (like we dont have to make the motors rotate opposite)
    public Command climberUp() {
        return Commands.run(
            () -> {
                if (Math.abs(climberLeft.getPosition().getValueAsDouble()) <= 65.0) {
                    climberLeft.set(-climberUpDownSpeed);
                    climberRight.set(climberUpDownSpeed);
                    CommandScheduler.getInstance().cancelAll();
                } else {
                    climberLeft.set(-0.015);
                    climberRight.set(0.015);
                }
            }
        );
    }

    public Command climberDown() {
        return Commands.run(
            () -> {
                if (Math.abs(climberLeft.getPosition().getValueAsDouble()) >= 5.0) {
                    climberLeft.set(climberUpDownSpeed);
                    climberRight.set(-climberUpDownSpeed);
                } else {
                    climberLeft.set(-0.015);
                    climberRight.set(0.015);
                }
            }
        );
    }

    public Command climberUpManual() {
        return Commands.run(
            () -> {
                climberLeft.set(-climberUpDownSpeed);
                climberRight.set(climberUpDownSpeed);
            }
        );
    }

    public Command climberDownManual() {
        return Commands.run(
            () -> {
                climberLeft.set(climberUpDownSpeed);
                climberRight.set(-climberUpDownSpeed);
            }
        );
    }

    public Command climberUpSlow() {
        return Commands.run(
            () -> {
                if (Math.abs(climberLeft.getPosition().getValueAsDouble()) <= 65.0) {
                    climberLeft.set(-climberUpDownSpeedSlow);
                    climberRight.set(climberUpDownSpeedSlow);
                } else {
                    climberLeft.set(-0.015);
                    climberRight.set(0.015);
                }
            }
        );
    }

    public Command climberDownSlow() {
        return Commands.run(
            () -> {
                if (Math.abs(climberLeft.getPosition().getValueAsDouble()) >= 5.0) {
                    climberLeft.set(climberUpDownSpeedSlow);
                    climberRight.set(-climberUpDownSpeedSlow);
                } else {
                    climberLeft.set(-0.015);
                    climberRight.set(0.015);
                }
            }
        );
    }

    public Command stopClimber() {
        return Commands.run(
            () -> {
                climberLeft.set(-0.015);
                climberRight.set(0.015);
            }
        );
    }
}
