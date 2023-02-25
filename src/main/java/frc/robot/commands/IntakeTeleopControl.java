/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.config.Constants;
import frc.robot.subsystems.Intake.State;
import frc.robot.utils.OI;

public class IntakeTeleopControl extends CommandBase {
	/**
	 * Creates a new IntakeTeleopControl.
	 */

	private OI oi;
	private Timer timer;
	private boolean firstTick;

	public IntakeTeleopControl() {
		// Use addRequirements() here to declare subsystem dependencies.
		oi = OI.getInstance();
		addRequirements(Robot.m_intake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		firstTick = true;
		timer = new Timer();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (!oi.shootButtonPressed()) {
			if (firstTick) {
				timer.start();
				firstTick = false;
			}
			// if (timer.get() < Constants.intakePreshotTime) {
			// 	Robot.m_intake.setState(State.PRESHOT);
			// } 
			 if (Robot.shooter.readyToFire()) {
				if (oi.intakeButtonPressed()) {
					Robot.m_intake.setState(State.INTAKE_AND_FEED);
				} else {
					Robot.m_intake.setState(State.FEEDING_SHOOTER);
				}
			} else {
				if (oi.intakeButtonPressed()) {
					Robot.m_intake.setState(State.BALL_PICKUP);
				} else {
					Robot.m_intake.setState(State.STOWED);
				}
			}
		} else {
			firstTick = true;
			if (oi.intakeButtonPressed()) { // intake ball
				Robot.m_intake.setState(State.BALL_PICKUP);
			} else {
				Robot.m_intake.setState(State.STOWED);
			}
		}

		// if (oi.getAButtonDriver()) { // intake ball
		// Robot.m_intake.setState(State.FEEDING_SHOOTER);
		// } else {
		// Robot.m_intake.setState(State.STOWED);
		// }
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.m_intake.setState(State.STOWED);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
