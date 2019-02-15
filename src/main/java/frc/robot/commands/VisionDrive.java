/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class VisionDrive extends Command {
  Robot _robot;
  public VisionDrive(Robot robot) {
    _robot = robot;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    _robot.driveTrain.Update_Limelight_Tracking();

        // double steer = m_Controller.getX(Hand.kRight);
        // double drive = -m_Controller.getY(Hand.kLeft);
        // boolean auto = m_Controller.getAButton();
        // steer *= 0.70;
        // drive *= 0.70;
        
        if (_robot.stick.getRawButton(4))
        {
          if (_robot.driveTrain.m_LimelightHasValidTarget)
          {
                _robot.driveTrain.ArcadeDrive(-_robot.driveTrain.m_LimelightDriveCommand,_robot.driveTrain.m_LimelightSteerCommand);
          }
          else
          {
                _robot.driveTrain.ArcadeDrive(0.0,0.0);
          }
        }
        else
        {
          _robot.driveTrain.ArcadeDrive(0.0,0.0);
        }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
