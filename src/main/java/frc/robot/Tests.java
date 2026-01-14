package frc.robot;

import frc.robot.subsystems.mechanisms.climber.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;


public class Tests {
    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();


    // Run tests and return the number of tests passed
    public int runTests(){
        int testsPassed = 0;
        
        double[] shooterVals = shooterSubsystem.calculateShooterValues(9, 10, -65);

        System.out.println("" + shooterVals[0]);
        System.out.println("" + shooterVals[1]);
        

        

        return testsPassed;
    }
}
