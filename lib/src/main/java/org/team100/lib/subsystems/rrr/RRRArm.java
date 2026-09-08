package org.team100.lib.subsystems.rrr;

import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.kinematics.rrr_se2.RRRFeasibility;
import org.team100.lib.kinematics.rrr_se2.RRRKinematicsPoE;
import org.team100.lib.subsystems.rn.PositionSubsystemRn;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

/**
 * NOTE: using the SE2 position API directly risks "flipping" in an uncontrolled
 * way.
 * 
 * Test your SE2 paths in simulation before using them on a real mechanism.
 */
public interface RRRArm extends PositionSubsystemSE2, PositionSubsystemRn<N3> {

    /** TODO: remove this */
    RRRKinematicsPoE kinematics();

    /** TODO: remove this */
    RRRFeasibility feasibility();

    /** Current measurement as config. */
    RRRConfig getConfig();

    RRRVelocity getVelocity();

    /**
     * Most-recent desired config, with limits applied.
     * TODO: apply limits upstream, remove this method
     */
    RRRConfig getConfigWithinLimits();

    /** TODO: remove this */
    RRRConfig config(Pose2d p);

    /** TODO: remove this */
    RRRVelocity qdot(RRRConfig q, VelocitySE2 xdot);

    /**
     * Imposes joint limits; returns the position and velocity actually used.
     */
    void set(RRRConfig q, RRRVelocity qdot, RRRAcceleration qddot);

    void stop();

    /** TODO: remove these (they're in kinematics) */
    double l1();

    double l2();

    double l3();
}
