/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class PID {
    private double p; //kP
    private double i; //kI
    private double d; //kD

    private double e; //current error
    private double prev_e; //previous error
    private double total_e; //cumulative error

    private double t; //current time
    private double last_t; //time of last iteration
    private double dt; //time between current and previous iteration
    private double de;

    private double target; //target value
    private double tolerance;

    public PID(double kp, double ki, double kd, double tol){
        p = kp;
        i = ki;
        d = kd;
        tolerance = tol;
        reset();
    }

    public void reset(){
        total_e = 0;
        prev_e = 0;
    }

    public void setTarget(double val) {
        if(val != target){
            reset();
        }
        target = val;
        e = target;
    }
    public double getTarget() {
        return target;
    }

    /**
     * Performs one iteration of a PID loop
     * @param observed measured value
     * @return unclamped output of the PID calculation
     */
    protected double step(double observed){
        double output = 0;

        t = Timer.getFPGATimestamp();
        dt = t - last_t;
        last_t = t;

        e = target - observed;
        de = e - prev_e;
        prev_e = e;
        total_e += e * dt;

        //the PID formula is output = weighted proportional error + weighted integrated error + weigthed differentiated error + feedforward
        output = p*e + i*total_e + d*de/dt;

        if(output > 1.0) output = 1.0;
        else if(output < -1.0) output = -1.0;

        return output;
    }

    /**
     * Performs one iteration of a PID loop
     * @param observed measured value
     * @return output clamped from -1 to 1 of the PID calculation
     */
    public double calculate(double observed){
        double output = step(observed);

        if(output > 1.0) output = 1.0;
        else if(output < -1.0) output = -1.0;

        return output;
    }

    public boolean isDone() {
		// close enough to target and not moving
		if (Math.abs(e) <= tolerance && Math.abs(de/dt) <= tolerance) {
			return true;
		} else {
			return false;
		}
	}
}