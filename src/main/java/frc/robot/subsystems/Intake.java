/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.config.Config;
import frc.robot.config.Constants;

/**
 * Add your docs here.
 */
public class Intake extends SubsystemBase {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private static Intake instance;
	private double rollerOutput;
	private double conveyorOutput;
	private VictorSPX mRoller;
	private VictorSPX mConveyor; 
	private State state;
	private Solenoid intakePiston; //extending solenoid


	public enum State{
		FEEDING_SHOOTER("FEEDING_SHOOTER"), 
		BALL_PICKUP("BALL_PICKUP"),
		INTAKE_AND_FEED("INTAKE_AND_FEED"),
		STOWED("STOWED"),
		PRESHOT("PRESHOT"),
		REVERSE("REVERSE"),
		DISABLED("DISABLED");
		
		private String name;
		private State(String nString){
			name = nString;
		}
		public String getName() {
			return name;
		}
	}

	public static Intake getInstance(){
		if(instance == null){
			instance = new Intake(Robot.m_cfg);
		}
		return instance;
	}

	private Intake(Config cfg)
	{
		rollerOutput = 0;
		conveyorOutput = 0;
		mRoller = cfg.getRoller();
		intakePiston = cfg.getIntakePiston();
		state = State.STOWED;
		mConveyor = cfg.getConveyor();
	}

	private void handleStates() {
			switch(state) {
		case FEEDING_SHOOTER:
			handleFEEDING_SHOOTER();
			break;
		case BALL_PICKUP:
			handleBALL_PICKUP();
			break;
		case INTAKE_AND_FEED:
			handleINTAKE_AND_FEED();
			break;
		case STOWED:
			handleSTOWED();   
			break;
		case PRESHOT:
			handlePRESHOT();
		case REVERSE:
			handleREVERSE();
		case DISABLED:
			handleDISABLED();
			break;
		}
	}
	
	public void setState(State desiredState)
	{
		this.state = desiredState;
	}

	public State getState()
	{
		return state;
	}

	//extended
	private void handleBALL_PICKUP()
	{
		intakePiston.set(true);
		setOutput(Constants.intakeRollerPower, 0);

	}
	
	//in
	private void handleFEEDING_SHOOTER()
	{
		intakePiston.set(false);
		setOutput(Constants.intakeRollerPower, Constants.intakeConveyorPower);
	}

	//out & feeding
	private void handleINTAKE_AND_FEED()
	{
		intakePiston.set(true);
		setOutput(Constants.intakeRollerPower, Constants.intakeConveyorPower);
	}


	//in
	private void handleSTOWED()
	{
		intakePiston.set(false);
		setOutput(0, 0);
	}

	// reverse
	private void handlePRESHOT() {
		intakePiston.set(false);
		setOutput(Constants.intakeRollerReverseOutput, Constants.intakeConveyorReverseOutput);
	}

	private void handleREVERSE() {
		setOutput(Constants.intakeRollerReverseOutput, Constants.intakeConveyorReverseOutput);
	}

	private void handleDISABLED() 
	{
		intakePiston.set(false);
		setOutput(0, 0);
	}

	public void setOutput(double output, double conveyorOutput) {
		this.rollerOutput = output;
		this.conveyorOutput = conveyorOutput;

	}

	// motor
	private void runRoller(double power) {
		mRoller.set(ControlMode.PercentOutput, power);
	}

	private void runConveyor(double power) {
		mConveyor.set(ControlMode.PercentOutput, power); 
	}
	
	@Override
	public void periodic(){
		handleStates();
		runRoller(rollerOutput);   
		runConveyor(conveyorOutput);
	}
}
