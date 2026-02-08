
package frc.robot.subsystems.motor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * MotorSubsystem provides utility methods for motion-related calculations used by
 * other subsystems and commands. It includes helpers to compute the signed angle
 * required to face a target and to estimate projectile launch parameters for
 * reaching a target at a given horizontal and vertical distance.
 *
 * <p>This class does not directly drive actuators; it exposes pure calculation
 * methods that callers can use when planning or aiming mechanisms.
 * 
 * In the future this should be moved to WhatTime
 */
public class MotorSubsystem extends SubsystemBase {
    /**
     * Calculates the signed angular difference (in degrees) required to rotate from a
     * reference heading to face a target described by a 2D position vector.
     *
     * The target bearing is computed as -atan2(y, x) (converted to degrees). The returned
     * value is (angle - targetBearing). Positive return values indicate a counterclockwise
     * rotation; negative values indicate a clockwise rotation.
     *
     * Notes:
     * - Units: degrees.
     * - Sign convention: positive = counterclockwise, negative = clockwise.
     * - The input 'angle' must use the same degrees/sign convention as the returned value.
     * - The returned angle is not normalized and may lie outside the [-180, 180] range;
     *   normalize if you require the smallest absolute rotation.
     * - Behavior is undefined if position is null or has fewer than two elements.
     *
     * Example:
     * - position = [0, 1], angle = 0:
     *   atan2(1, 0) = +90° → targetBearing = -90° → return = 0 - (-90) = +90° (rotate 90° CCW)
     *
     * @param position a 2-element array representing the target vector [x, y]
     * @param angle    the current heading in degrees (positive = CCW)
     * @return the signed angle in degrees to rotate from 'angle' to face 'position'
     */
    public double degreesToFace(double[] position, double angle){ // negative is clockwise positive is counterclockwise
        return angle - (
            -Math.atan2(position[1], position[0])
        ) * 180 / Math.PI;
    }
    
    /**
     * Calculates the required launch speed and angle to hit a target at a given (X, Y) distance,
     * given an initial entry angle. This uses projectile motion equations assuming no air resistance.
     *
     * The method returns an array: [launchSpeed (m/s), launchAngle (degrees)].
     *
     * @param distanceX   Horizontal distance to the target (meters)
     * @param distanceY   Vertical distance to the target (meters)
     * @param enterAngle  Initial entry angle in degrees (0 = horizontal)
     * @return            double[] where index 0 is launch speed (m/s), index 1 is launch angle (degrees)
     *
     * Note: The calculation assumes Earth's gravity (9.81 m/s^2) and that the projectile is launched from the origin.
     *       The formula is derived from kinematic equations for projectile motion.
     *       Behavior is undefined if the input values result in a negative value under the square root.
     */
    public double[] calculateTrajectory(double distanceX, double distanceY, double enterAngle){
        // https://www.desmos.com/calculator/aavfklnkvh


        enterAngle = enterAngle * Math.PI / 180;
        double outputAngle = Math.atan(2 * distanceY / distanceX - Math.tan(enterAngle));

        double launchSpeed = Math.sqrt(
            ( 9.81 * distanceX * distanceY ) / 
            ( 2 * Math.cos(outputAngle) * Math.cos(outputAngle) 
            * ( distanceX * Math.tan(outputAngle) - distanceY ) 
        ));
        
        return new double[]{launchSpeed, outputAngle * 180 / Math.PI};
        
    }

}
