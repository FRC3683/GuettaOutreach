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
public class PIDF extends PID {
    private double fV; //velocity feed forward

    public PIDF(double kp, double ki, double kd, double kf, double tol) {
		super(kp, ki, kd, tol);
		fV = kf;
	}

    /**
     * Performs one iteration of a PIDF loop
     * @param observed measured value
     * @return unclamped output of the PIDF calculation
     */
    protected double step(double observed){
        double output = super.step(observed) + fV * super.getTarget();
        return output;
    }

    /**
     * Performs one iteration of a PIDF loop
     * @param observed measured value
     * @return output clamped from -1 to 1 of the PIDF calculation
     */
    public double calculate(double observed){
        double output = step(observed);

        if(output > 1.0) output = 1.0;
        else if(output < -1.0) output = -1.0;

        return output;
    }
    public void setfV(double fV ){
        this.fV = fV;
    }
}