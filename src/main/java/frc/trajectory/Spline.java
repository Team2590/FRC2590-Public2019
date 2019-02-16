/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.trajectory;

import java.util.ArrayList;

/**
 * Array of Points that define a cubic hermite spline path
 */
public class Spline {

    private ArrayList<Point> points;

    private double t_step, distance;

    public Spline(double x0 , double y0, double theta0, double t_step) {
        points = new ArrayList<Point>();
        this.t_step = t_step;

        for(int t = 0; t < 1; t += t_step){ 
            points.add(new Point(t, x0, y0, theta0));
            int index = (int)(t/t_step);
            distance += 2 * points.get(index).get_integrand();
        }

        distance = 0.5 * t_step * (distance - points.get(0).get_integrand() - points.get((int)(1/t_step)).get_integrand());
    }

    public double getDistance() {
        return distance;
    }

    //maybe only return a single point, idk
    public ArrayList<Point> getPoints() {
        return points;
    }
}
