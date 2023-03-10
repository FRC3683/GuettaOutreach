/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class shootBallsAuto extends CommandBase {
	/**
	 * Creates a new shootBallsAuto.
	 */

	private int ballsShot;
	private int ballsToShoot;	

	public shootBallsAuto(int balls) {
		// Use addRequirements() here to declare subsystem dependencies.
		ballsToShoot = balls;
		addRequirements(Robot.shooter);
		addRequirements(Robot.m_intake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		ballsShot = 0;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (!Robot.shooter.foundDistance()) {
			Robot.shooter.setState(Shooter.State.FINDING_DISTANCE);
		} else if (!Robot.shooter.readyToFire()) {
			Robot.shooter.setStateReving();
		} else {
			Robot.m_intake.setState(Intake.State.FEEDING_SHOOTER);
			Robot.shooter.setStateShooting();
		}
		if(Robot.shooter.shotBall()) {
			ballsShot++;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return ballsShot >= ballsToShoot;
	}
}
