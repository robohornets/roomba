package frc.robot.subsystems.mechanisms.feeder;


public enum FeederState {
    /** Runs all motors in the feeder subsystem in towards the shooter. */
    ALL_FEEDER_IN,
    /** Runs all motors in the feeder subsystem out away from the shooter. */
    ALL_FEEDER_OUT,
    /** Runs the shooter feed and feeder feeder motors to control feeding to the shooter. */
    SHOOTER_FEED_IN,
    /** Runs the shooter feed and feeder feeder motors out to remove balls from the shooter in the event they get stuck. */
    SHOOTER_FEED_OUT,
    /** Stops all motors in the feeder subsystem. */
    OFF;
}
