/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.MathUtils;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.Robot;
import frc.robot.utils.PID;

/**
 * Add your docs here.
 */
public class Eye {
	private static Eye instance;

	/**
	 * This function implements a simple method of generating driving and steering
	 * commands based on the tracking data from a Rasperry Pi writing to a network
	 * table under the name Eye.
	 */
	public static Eye getInstance() {
		if (instance == null) {
			instance = new Eye();
		}
		return instance;
	}

	private Eye(){
	}

	public double getDistanceFromTarget() {
		double[] camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[6]);
		double x = camtran[0];
		double z = camtran[2];
		return Math.sqrt(x*x+z*z);//the limelight returns x & z, we need hypotenuse
	}

	public double getHeadingCorrection(){
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
	}

	public boolean onTarget() {
		return MathUtils.closeEnough(getHeadingCorrection(), 0, Constants.headingTolerance);
	}

	public boolean validTarget(){
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) > 0.5;
	}
}