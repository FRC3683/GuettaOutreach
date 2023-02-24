/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.State;
import frc.robot.utils.OI;

public class ShooterTeleopCommand extends CommandBase {
	/**
	 * Creates a new ShooterTeleopCommand.
	 */
	private OI m_OI;

	public ShooterTeleopCommand() {
		m_OI = OI.getInstance();
		addRequirements(Robot.shooter);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_OI.shootButtonPressed()) {
			// if (!Robot.shooter.foundDistance()) {
			// 	Robot.shooter.setState(Shooter.State.FINDING_DISTANCE);
			// 	m_OI.rumbleDriver(0);
			// } else if (!Robot.shooter.inShootingRange()) {
			// 	Robot.shooter.setStateReving();
			// 	m_OI.rumbleDriver(0.3);
			if (!Robot.shooter.readyToFire()) {
				//Robot.shooter.setStateReving();
				Robot.shooter.setState(State.REVING_FAR);
				m_OI.rumbleDriver(0);
			} else {
				//Robot.shooter.setStateShooting();
				Robot.shooter.setState(State.SHOOTING_FAR);
				m_OI.rumbleDriver(0);
			}
		} else {
			m_OI.rumbleDriver(0);
			Robot.shooter.setState(Shooter.State.STOWED);
		}
		// if(m_OI.getLeftBumperDriver()){
		// 	Robot.shooter.setState(State.REVING_FAR);
		// 	//Robot.shooter.setOutput(0.5);
		// 	// Robot.shooter.setOutput(m_OI.getYLeftDriver());
		// }else{
		// 	Robot.shooter.setState(Shooter.State.STOWED);
		// 	Robot.shooter.setOutput(0);
		// }
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
