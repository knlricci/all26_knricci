package org.team100.lib.geometry.rrr;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record RRRAcceleration(
        double q1ddot,
        double q2ddot,
        double q3ddot) {

    public static RRRAcceleration fromVector(Matrix<N3, N1> v) {
        return new RRRAcceleration(
                v.get(0, 0),
                v.get(1, 0),
                v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(
                q1ddot,
                q2ddot,
                q3ddot);
    }

    public static RRRAcceleration solve(RRRConfig x0, RRRConfig x1, RRRVelocity v0, double dt) {
        return RRRAcceleration.fromVector(
                x1.toVector().minus(x0.toVector()).minus(v0.toVector().times(dt)).times(2 / (dt * dt)));
    }

    @Override
    public String toString() {
        return String.format("%6.3f %6.3f %6.3f", q1ddot, q2ddot, q3ddot);
    }
}
