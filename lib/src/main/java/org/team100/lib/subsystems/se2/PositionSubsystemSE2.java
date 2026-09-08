package org.team100.lib.subsystems.se2;

import org.team100.lib.state.ControlSE2;

/**
 * A planar subsystem controlled by position.
 * 
 * For example, a multi-DOF arm.
 * 
 * It would be possible for the swerve drive to be controlled
 * this way (folding position feedback into the subsystem),
 * but since we manually control it by velocity, we don't
 * do that.
 */
public interface PositionSubsystemSE2 extends SubsystemSE2 {

    /**
     * Position, velocity, and acceleration in SE2.
     * 
     * Subsystems are expected to compute the (generalized)
     * force required to meet this setpoint, using the dynamics
     * of the mechanism.
     * 
     * @param setpoint for the next timestamp
     */
    void set(ControlSE2 setpoint);
}