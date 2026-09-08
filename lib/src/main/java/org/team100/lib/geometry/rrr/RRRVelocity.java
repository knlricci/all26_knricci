package org.team100.lib.geometry.rrr;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * qdot in rad/s
 */
public record RRRVelocity(
        double q1dot,
        double q2dot,
        double q3dot) {

    public static RRRVelocity fromVector(Matrix<N3, N1> v) {
        return new RRRVelocity(
                v.get(0, 0),
                v.get(1, 0),
                v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(
                q1dot,
                q2dot,
                q3dot);
    }

    public RRRVelocity times(double a) {
        return new RRRVelocity(a * q1dot, a * q2dot, a * q3dot);
    }

    /**
     * a = (v1 - v0) / dt
     */
    public RRRAcceleration accel(RRRVelocity prev, double dt) {
        return RRRAcceleration.fromVector(
                toVector().minus(prev.toVector()).div(dt));
    }

    /**
     * v1 = v0 + a dt
     */
    public static RRRVelocity evolve(RRRVelocity v0, RRRAcceleration a, double dt) {
        return RRRVelocity.fromVector(v0.toVector().plus(a.toVector().times(dt)));
    }

    @Override
    public String toString() {
        return String.format("%6.3f, %6.3f, %6.3f", q1dot, q2dot, q3dot);
    }

}
