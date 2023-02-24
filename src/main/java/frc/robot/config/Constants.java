/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.config;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double shooterTicksPerRev = 4096; //1024; from datasheet 
    public static final double shooterRadPerPulse = 2*Math.PI/shooterTicksPerRev; // TODO insert real value based on ticks in encoder and convert to rad per sec.
    public static final double shooterPulsePerRad = 1.0 / shooterRadPerPulse;
    public static final double shooter100msPerS = 10.0;
    public static final double testSpeed = 500;
    public static final double shooterHoodAngleNear = 45;
    public static final double shooterHoodAnglefar = 30;
    public static final double shooterNearMinDistance = 0; // TODO insert real values
    public static final double shooterNearMaxDistance = 0;
    public static final double shooterFarMinDistance = 0;
    public static final double shooterFarMaxDistance = 0;
    
    public static final double shooterP = 0.225;//.075;//0.0025;
    public static final double shooterI = 0.001;
    public static final double shooterD = 0;//0.0002;
    public static final int shooterIZone = 950;//ticks per 100 ms

    public static final double shooterS = 0.47; // TODO insert real values [V]
    public static final double shooterV = 0.0206965755;//0.0175828125;//1/79.57105405;//(18730/2)/12; //[V*s/rad]
    public static final double shooterA = 0; //[V*s*s/rad]
    
    public static final double shooterMaxVoltage = 12;
    public static final double shooterChangeInSpeedShot = 1; // TODO add real value
    public static final String shooterFile = ""; // TODO add file path

    public static final double intakeRollerPower = -0.85; 
    public static final double intakeConveyorPower = 0.85; 
    public static final double intakeConveyorReverseOutput = -0.85;
    public static final double intakeRollerReverseOutput = 0.85; // still intaking
    public static final double intakePreshotTime = 0.2;

    public static final double driveLeftKP = 0;
    public static final double driveLeftKI = 0;
    public static final double driveLeftKD = 0;
    public static final double driveLeftTol = 0;
    public static final double driveLeftDTol = 0;
    public static final double driveRightKP = 0;
    public static final double driveRightKI = 0;
    public static final double driveRightKD = 0;
    public static final double driveRightTol = 0;
    public static final double driveRightDTol = 0;
    public static final double driveTurnKP = 0;
    public static final double driveTurnKI = 0;
    public static final double driveTurnKD = 0;
    public static final double driveTurnTol = 0;
    public static final double driveTurnDTol = 0;
    public static final double trackWidth = 21.345;
    public static final double period = 0.02;
	  public static final double driveTicksPerInchLeft = 0;
    public static final double driveTicksPerInchRight = 0;
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double kMaxSpeedMetersPerSecond = 0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0;
    public static final double kRamseteB = 0;
    public static final double kRamseteZeta = 0;
}
