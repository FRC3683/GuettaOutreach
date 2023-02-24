/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.config.Constants;
import frc.robot.utils.BNO055;
import frc.robot.utils.DaveAccelerometer;
import frc.robot.utils.MathUtils;
import frc.robot.vision.Eye;

public class DriveTrain extends SubsystemBase {
	private static DriveTrain instance;
	// drive motor objects
	private CANSparkMax mLeft1;
	private CANSparkMax mLeft2;
	private CANSparkMax mRight1;
	private CANSparkMax mRight2;
	// drive encoder objects
	private Encoder leftEnc;
	private Encoder rightEnc;
	// gyro/accelerometer objects
	private BNO055 mGyro;
	private DaveAccelerometer mAccelerometer;
	// motor output variables
	private double leftOutput;
	private double rightOutput;
	private double targetLeftVel;
	private double targetRightVel;
	// PID objects
	private PIDController leftPID;
	private PIDController rightPID;
	private PIDController turnPID;
	// Vision
	private Eye eye;
	// Kinimatics variables for calculating wheel speeds from desired robot speeds
	private double vx;
	private final double vy = 0;
	private double omega;
	private DifferentialDriveKinematics kinematics;
	private DifferentialDriveOdometry odometry;
	// shuffleboard and networktables variables
	private ShuffleboardTab tab;
	private NetworkTableEntry headingEntry;
	private NetworkTableEntry rollEntry;
	private NetworkTableEntry pitchEntry;
	private NetworkTableEntry encLeftEntry;
	private NetworkTableEntry encRightEntry;
	private NetworkTableEntry leftOutEntry;
	private NetworkTableEntry rightOutEntry;
	private NetworkTableEntry stateEntry;
	private NetworkTableEntry targetEntry;
	private NetworkTableEntry offsetEntry;
	private NetworkTableEntry onTargetEntry;
	// gyro offset variables
	private double offsetHeadingAngle;
	private double offsetPitchAngle;
	private double offsetRollAngle;
	private double targetAngle;
	private boolean foundAngle;

	public DriveTrain() {
		state = State.DISABLED;

		mLeft1 = Config.getDriveMotorLeft1();
		mLeft2 = Config.getDriveMotorLeft2();
		mRight1 = Config.getDriveMotorRight1();
		mRight2 = Config.getDriveMotorRight2();

		//leftEnc = Config.getDriveLeftEnc();
		//rightEnc = Config.getDriveRightEnc();
		//mGyro = Config.getDriveGyro();
		mAccelerometer = DaveAccelerometer.getInstance();

		kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.trackWidth));
		new DifferentialDriveOdometry(new Rotation2d(getAbsoluteHeading()), 0, 0);
		// offsets
		offsetHeadingAngle = 0.0;
		offsetPitchAngle = 0.0;
		offsetRollAngle = 0.0;

		//tab = Shuffleboard.getTab("Drive");

		leftPID = new PIDController(Constants.driveLeftKP, Constants.driveLeftKI, Constants.driveLeftKD,
				Constants.period);
		leftPID.setTolerance(Constants.driveLeftTol, Constants.driveLeftDTol);
		rightPID = new PIDController(Constants.driveLeftKP, Constants.driveRightKI, Constants.driveRightKD,
				Constants.period);
		rightPID.setTolerance(Constants.driveRightTol, Constants.driveRightDTol);
		turnPID = new PIDController(Constants.driveTurnKP, Constants.driveTurnKI, Constants.driveTurnKD,
				Constants.period);
		turnPID.setTolerance(Constants.driveTurnTol, Constants.driveTurnDTol);

		leftOutput = 0.0;
		rightOutput = 0.0;

		//eye = Eye.getInstance();
		foundAngle = false;

		//tab.add("Left Encoder", leftEnc);
		//tab.add("Right Encoder", rightEnc);

		// headingEntry = tab.add("Heading", 0).getEntry();
		// rollEntry = tab.add("roll", 0).getEntry();
		// pitchEntry = tab.add("Pitch", 0).getEntry();
		// encLeftEntry = tab.add("LeftEnc", 0).getEntry();
		// encRightEntry = tab.add("RightEnc", 0).getEntry();
		// stateEntry = tab.add("State", "disabled").getEntry();
		// leftOutEntry = tab.add("Thrust", leftOutput).getEntry();
		// rightOutEntry = tab.add("Rotate", rightOutput).getEntry();
		// offsetEntry = tab.add("Offset", offsetHeadingAngle).getEntry();
		// onTargetEntry = tab.add("onTargetAngle", false).getEntry();
	}

	public static DriveTrain getInstance() {
		if (instance == null) {
			instance = new DriveTrain();
		}
		return instance;
	}

	public enum State {
		CLOSED_LOOP("CLOSED_LOOP"), // for use in auto
		OPEN_LOOP("OPEN_LOOP"), // driver controled (for use in teleop)
		FINDING_ANGLE("FINDING_ANGLE"), // finding angle to turn to from limelight
		TURNING("TURNING"), // for when turning to an angle
		DISABLED("DISABLED"); // when robot is disabled

		private String name;

		private State(String nString) {
			name = nString;
		}

		public String getName() {
			return name;
		}
	}

	private State state;

	private void handleStates() {
		switch (state) {
		case CLOSED_LOOP:
			handleCLOSED_LOOP();
			break;
		case OPEN_LOOP:
			handleOPEN_LOOP();
			break;
		case FINDING_ANGLE:
			handleFINDING_ANGLE();
			break;
		case TURNING:
			handleTURNING();
			break;
		case DISABLED:
			handleDISABLED();
			break;
		}
	}

	private void handleFINDING_ANGLE() {
		brakeMode();

		setTargetAngle(0);

        if(eye.validTarget()) {
            setTargetAngle(getAbsoluteHeading() + eye.getHeadingCorrection()); //Ange did this if its wrong blame them
            foundAngle = true;
        } else {
            foundAngle = false;
        }
	}

	private void handleTURNING() {
		brakeMode();

		double rotate = 0.0;
		if (!onTargetAngle()) {
			rotate = turnPID.calculate(getAbsoluteHeading());
		}
		setLeft(MathUtils.calcLeftDrive(0.0, rotate));
		setRight(MathUtils.calcRightDrive(0.0, rotate));
	}

	void handleCLOSED_LOOP() {
		brakeMode(); // for more acurate autonomus movements
		foundAngle = false;
	}

	void handleOPEN_LOOP() {
		coastMode(); // motion by driver can be smoother (slow is smooth and smooth is fast)
		foundAngle = false;
	}

	void handleDISABLED() {
		// motors cannot be under power when disabled
		coastMode();
		setLeft(0.0);
		setRight(0.0);
		foundAngle = false;
	}

	// speed/motor functions (make robot go vroom vroom zoom zoom)
	public void setLeft(double power) {
		leftOutput = power;
	}

	public void setRight(double power) {
		rightOutput = power;
	}

	public void setTargetLeftVel(double InchesPerSecond) {
		targetLeftVel = InchesPerSecond;
	}

	public void setTargetRightVel(double InchesPerSecond) {
		targetRightVel = InchesPerSecond;
	}

	public void setWheelSpeedsFromChassisSpeed(ChassisSpeeds speeds) {
		// set left/right velocities from overall robot velocities
		DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
		setTargetLeftVel(Units.metersToInches(wheelSpeeds.leftMetersPerSecond));
		setTargetRightVel(Units.metersToInches(wheelSpeeds.rightMetersPerSecond));
	}

	private void runLeft(double power) {
		mLeft1.set(power);
		mLeft2.set(power);
	}

	private void runRight(double power) {
		mRight1.set(power);
		mRight2.set(power);
	}

	public void runMotorsVolts(double leftVolts, double rightVolts) {
		mLeft1.setVoltage(leftVolts);
		mLeft2.setVoltage(leftVolts);
		mRight1.setVoltage(-rightVolts);
		mRight2.setVoltage(-rightVolts);
	}

	// modes
	private void coastMode() {
		// allows for smoother, less accurate movement
		mLeft1.setIdleMode(IdleMode.kCoast);
		mLeft2.setIdleMode(IdleMode.kCoast);
		mRight1.setIdleMode(IdleMode.kCoast);
		mRight2.setIdleMode(IdleMode.kCoast);
	}

	private void brakeMode() {
		// allows for less smooth, more acurate movement
		mLeft1.setIdleMode(IdleMode.kBrake);
		mLeft2.setIdleMode(IdleMode.kBrake);
		mRight1.setIdleMode(IdleMode.kBrake);
		mRight2.setIdleMode(IdleMode.kBrake);
	}

	// sensor getters and zero functions
	private int getLeftRaw() {
		return leftEnc.getRaw();
	}

	private int getRightRaw() {
		return rightEnc.getRaw();
	}

	// gives encoder speeds in inches per second
	private double getLeftSpeed() {
		return leftEnc.getRate();
	}

	private double getRightSpeed() {
		return rightEnc.getRate();
	}

	public double getHeading() {
		return 0.0;//mGyro.getHeading() - offsetHeadingAngle;
	}

	public double getAbsoluteHeading() {
		return Math.IEEEremainder(getHeading(), 360.0);
	}

	public double getRoll() {
		return mGyro.getVector()[1] - offsetRollAngle;
	}

	public double getPitch() {
		return mGyro.getVector()[2] - offsetPitchAngle;
	}

	public double getDistance() {
		return (MathUtils.average(leftEnc.getDistance(), rightEnc.getDistance()));
	}

	public State getState() {
		return state;
	}

	public void setState(State targetState) {
		this.state = targetState;
	}

	public void zeroGyro() {
		System.out.println("Gyro Zeroed");
		offsetHeadingAngle = getHeading();
		offsetPitchAngle = getPitch();
		offsetRollAngle = getRoll();
	}

	public void zeroEncoders() {
		leftEnc.reset();
		rightEnc.reset();
	}

	public void zeroAccelerometer() {
		mAccelerometer.resetVelocities();
	}

	// send data to dasboard for drivers to see
	private void sendToDashboard() {
	//	headingEntry.setDouble(getHeading());
		encLeftEntry.setDouble(getLeftRaw());
		encRightEntry.setDouble(getRightRaw());
		offsetEntry.setDouble(offsetHeadingAngle);
		stateEntry.setString(state.getName());
	}

	public DifferentialDriveKinematics getKinematics() {
		return kinematics;
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public PIDController getLeftPID() {
		return leftPID;
	}

	public PIDController getRightPID() {
		return rightPID;
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
	}

	private boolean onTargetAngle() {
		return turnPID.atSetpoint();
	}

	public double getTargetAngle() {
		return targetAngle;
	}

	public void setTargetAngle(double targetAngle) {
		this.targetAngle = targetAngle;
		turnPID.setSetpoint(targetAngle);
	}

	public boolean foundAngle(){
		return foundAngle;
	}
	// at regular intervals (every 20ms) iterate through our state machine and send
	// data to the drivers via dashboard
	// set motor speeds at consistant intervals to ensure reliability
	@Override
	public void periodic() {
		handleStates();
		//sendToDashboard();
		mAccelerometer.updateVelocities();

		runLeft(leftOutput);
		runRight(rightOutput);
	}
}