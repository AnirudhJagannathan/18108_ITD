package org.firstinspires.ftc.teamcode.geometry;

import kotlin.jvm.JvmStatic;

public class Angle {
    private static double TAU = Math.PI * 2;

    /**
     * Returns [angle] clamped to `[0, 2pi]`.
     *
     */
    @JvmStatic
    public static double norm(double angle) {
        double modifiedAngle = angle % TAU;

        modifiedAngle = (modifiedAngle + TAU) % TAU;

        return modifiedAngle;
    }

    /**
     * Returns [angleDelta] clamped to `[-pi, pi]`.
     *
     * @param angleDelta angle delta in radians
     */
    @JvmStatic
    public static double normDelta(double angleDelta) {
        double modifiedAngleDelta = norm(angleDelta);

        if (modifiedAngleDelta > Math.PI) {
            modifiedAngleDelta -= TAU;
        }

        return modifiedAngleDelta;
    }
}
