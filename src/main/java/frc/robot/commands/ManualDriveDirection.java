/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Helpers;
import frc.robot.Robot;
import frc.robot.TurnPid;

/**
 * An example command.  You can replace me with your own command.
 */
public class ManualDriveDirection extends Command {
  private Robot _robot; 
  private int direction;
  private TurnPid _turnPid;
  private int stoppedCounter = 0;
  private boolean testCompleted = false;

  public ManualDriveDirection(Robot robot, int angle) {
    // Use requires() here to declare subsystem dependencies
    _robot = robot;
    requires(_robot.driveTrain);
    direction = angle;
    _turnPid = new TurnPid(_robot);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _turnPid.SetTargetAngle(direction);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double turnPower = _turnPid.GetAnglePidOutput(Helpers.ConvertYawToHeading(_robot.navX.getYaw()));
    _robot.driveTrain.Move(turnPower, turnPower);
    if (turnPower == 0){
      stoppedCounter ++;
    }
    else{
      stoppedCounter = 0;
    }
    if (stoppedCounter > 5){
      testCompleted = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return testCompleted;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    _robot.driveTrain.Move(0,0);    
  }
}
