/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * Add your docs here.
 */
public class PIDFA extends PIDF {
    private double fA; //acceleration feed forward

    public PIDFA(double kp, double ki, double kd, double kfv, double kfa, double tol){
        super(kp, ki, kd, kfv, tol);
        fA = kfa;
        reset();
    }

    /**
     * Performs one iteration of a PIDFA loop
     * @param observed measured value
     * @return unclamped output of the PIDFA calculation
     */
    protected double step(double observed, double targetAccel){
        double output = super.step(observed) + fA*targetAccel;
        return output;
    }

    /**
     * Performs one iteration of a PID loop
     * @param observed measured value
     * @param targetAccel target acceleration
     * @return output clamped from -1 to 1 of the PID calculation
     */
    public double calculate(double observed, double targetAccel){
        double output = step(observed, targetAccel);
        
        if(output > 1.0) output = 1.0;
        else if(output < -1.0) output = -1.0;

        return output;
    }
}