package org.team340.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.team340.robot.subsystems.Shooter;

/**
 * This class stores our lookup tables for the {@link Shooter}
 * subsystem to utilize when converting hub distance to motor setpoints.
 */
public final class ShootParams {

    /**
     * The ball's time of flight, in seconds. All entries in
     * our lookup tables below are tuned for this constant.
     */
    public static final double TOF = 1.0; // wow 1 second thats really cool

    /**
     * Our shooter velocity lookup table.
     * Maps hub distance (meters) to shooter velocity (rotations/second).
     */
    public static final InterpolatingDoubleTreeMap shooterVelocityMap;

    static {
        // Data obtained from empirical testing.
        final DataPoint[] dataPoints = {
            new DataPoint(1.51, 34.0),
            new DataPoint(2.00, 39.0),
            new DataPoint(2.50, 41.0),
            new DataPoint(2.99, 39.0),
            new DataPoint(3.51, 44.0),
            new DataPoint(3.96, 46.0),
            new DataPoint(4.59, 48.5),
            new DataPoint(5.15, 53.0),
            // Outside of alliance zone
            new DataPoint(6.15, 58.0),
            new DataPoint(8.39, 68.0),
            new DataPoint(9.71, 75.0),
            new DataPoint(10.12, 87.0)
        };

        // Create and populate our lookup table.
        shooterVelocityMap = new InterpolatingDoubleTreeMap();
        for (final DataPoint dataPoint : dataPoints) {
            shooterVelocityMap.put(dataPoint.distance, dataPoint.shooterVelocity);
        }
    }

    /**
     * A data point for our lookup table. Contains the shooter velocity
     * needed to make a shot from a given distance.
     * @param distance The robot's distance to the hub, in meters.
     * @param shooterVelocity The velocity of the shooter wheels, in rotations/second.
     */
    private record DataPoint(double distance, double shooterVelocity) {}

    private ShootParams() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
