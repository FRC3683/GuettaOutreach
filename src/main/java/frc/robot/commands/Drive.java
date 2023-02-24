/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain.State;
import frc.robot.utils.MathUtils;
import frc.robot.utils.OI;

public class Drive extends CommandBase {
  private OI oi;
  private double Y;
  private double X;
  private double leftOutput;
  private double rightOutput;

  public enum DriveMode {
    ARCADE_ONE_STICK, ARCADE_TWO_STICK, TANK;
  }

  DriveMode dMode;

  public Drive(DriveMode mode) {
    addRequirements(Robot.m_driveTrain);
    dMode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oi = OI.getInstance();
    Y = 0;
    X = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(oi.getRightTriggerDriver() > 0.5){
      if(!Robot.m_driveTrain.foundAngle()){
        Robot.m_driveTrain.setState(State.FINDING_ANGLE);
        //TODO: Vibrate Controller until angle is found
        //When angle is found so is distance, so we know the shooter can start reving after here
      } else {
        Robot.m_driveTrain.setState(State.TURNING);
        //TODO: Shooter should shoot when we are on target and at correct rev speed
      }
    } else {
      Robot.m_driveTrain.setState(State.OPEN_LOOP);
    }

    if (Robot.m_driveTrain.getState() == State.OPEN_LOOP) {
      switch (dMode) {
      case ARCADE_ONE_STICK:
        X = MathUtils.squaredInput(oi.getXLeftDriver());
        Y = MathUtils.squaredInput(oi.getYLeftDriver());
        leftOutput = MathUtils.calcLeftDrive(Y, X);
        rightOutput = MathUtils.calcRightDrive(Y, X);
        break;
      case ARCADE_TWO_STICK:
        X = MathUtils.squaredInput(oi.getXRightDriver());
        Y = MathUtils.squaredInput(oi.getYLeftDriver());
        leftOutput = MathUtils.calcLeftDrive(Y, X);
        rightOutput = MathUtils.calcRightDrive(Y, X);
        break;
      case TANK:
        X = MathUtils.squaredInput(oi.getYRightDriver());
        Y = MathUtils.squaredInput(oi.getYLeftDriver());
        leftOutput = Y;
        rightOutput = -X;
        break;
      }
    }
    Robot.m_driveTrain.setLeft(-leftOutput);
    Robot.m_driveTrain.setRight(-rightOutput);
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
