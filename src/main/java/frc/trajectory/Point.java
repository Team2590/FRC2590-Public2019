/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.trajectory;

/**
 * Add your docs here.
 */
public class Point {

    private double t, x, y, dxdt, dydt, d2xdt2, d2ydt2, heading, r_curve, integrand;

    /**
     * Parametrized point on a spline for a given t step
     * 
     * @param t      step along the path
     * @param x0     initial x coordinate
     * @param y0     initial y coordinate
     * @param theta0 initial heading
     */
    public Point(double t, double x0, double y0, double theta0) {
        this.t = t;

        // variables to make legibility simpler
        double t3 = Math.pow(t, 3);
        double t2 = Math.pow(t, 2);
        double sin_theta0 = Math.sin(theta0);
        double cos_theta0 = Math.cos(theta0);

        this.x = x0 * (2 * t3 - 3 * t2 + 1) + cos_theta0 * (t3 - 2 * t2 + t) + (t3 - t2);

        this.y = y0 * (2 * t3 - 3 * t2 + 1) + sin_theta0 * (t3 - 2 * t2 + t);

        this.dxdt = x0 * (6 * t2 - 6 * t) + cos_theta0 * (3 * t2 - 4 * t + 1) + (3 * t2 - 2 * t);

        this.dydt = y0 * (6 * t2 - 6 * t) + sin_theta0 * (3 * t2 - 4 * t + 1);

        this.d2xdt2 = x0 * (12 * t - 6) + cos_theta0 * (6 * t - 4) + (6 * t - 2);

        this.d2ydt2 = y0 * (12 * t - 6) + sin_theta0 * (6 * t - 4);

        this.heading = Math.atan2(dydt, dxdt);

        this.r_curve = Math.pow(Math.pow(dxdt, 2) + Math.pow(dydt, 2), 1.5) / Math.abs(dxdt * d2ydt2 - dydt * d2xdt2);

        this.integrand = Math.hypot(dxdt, dydt);
    }

    public double get_t() {
        return t;
    }

    public double get_x() {
        return x;
    }

    public double get_y() {
        return y;
    }

    public double get_dxdt() {
        return dxdt;
    }

    public double get_dydt() {
        return dydt;
    }

    public double get_d2xdt2() {
        return d2xdt2;
    }

    public double get_d2ydt2() {
        return d2ydt2;
    }

    public double get_heading() {
        return heading;
    }

    public double get_r_curve() {
        return r_curve;
    }

    public double get_integrand() {
        return integrand;
    }
}
