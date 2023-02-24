/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;
import frc.robot.config.Constants;

public class FollowTrajectory extends RamseteCommand {
	/**
	 * Creates a new FollowTrajectory.
	 */
	public FollowTrajectory(Trajectory path) {
		super(path, Robot.m_driveTrain::getPose, new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
				new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
						Constants.kaVoltSecondsSquaredPerMeter),
				Robot.m_driveTrain.getKinematics(), Robot.m_driveTrain::getWheelSpeeds, Robot.m_driveTrain.getLeftPID(),
				Robot.m_driveTrain.getRightPID(),
				// RamseteCommand passes volts to the callback
				Robot.m_driveTrain::runMotorsVolts, Robot.m_driveTrain);

		// Run path following command, then stop at the end.
		this.andThen(() -> Robot.m_driveTrain.runMotorsVolts(0, 0));
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
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
