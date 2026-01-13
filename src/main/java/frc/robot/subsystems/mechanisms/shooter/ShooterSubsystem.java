package frc.robot.subsystems.mechanisms.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // Just the base setup of the class, calculations will be done here (or in this folder) to shoot the fuel


    // TODO: VERY IMPORTANT!!!! Put the correct device ids
    public final TalonFX climberLeft = new TalonFX(-1);
    public final TalonFX climberRight = new TalonFX(-2);
}
