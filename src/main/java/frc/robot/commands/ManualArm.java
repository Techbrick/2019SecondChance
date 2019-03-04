/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.Helpers;
import frc.robot.Robot;
// import frc.robot.WristPid;

public class ManualArm extends Command {
  private Robot _robot;
  // private WristPid wristy;
  // private boolean on = false;

  public ManualArm(Robot r) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    _robot = r;
    requires(_robot.arm_subsystem);
    
    // wristy = new WristPid(_robot);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // if(_robot.stick.getRawButton(5)) {
    //   _robot.arm_subsystem.rotate(-1);
    // } else if(_robot.stick.getRawButton(6)) {
    //   _robot.arm_subsystem.rotate(1);
    // } else {
    //   _robot.arm_subsystem.rotate(0);
    // }

    // if(_robot.stick.getRawButton(3)) {
    //   _robot.arm_subsystem.rotateWrist(-1);
    // }
    // else if(_robot.stick.getRawButton(4)) {
    //   _robot.arm_subsystem.rotateWrist(1);
    // }
    // else {
    //   _robot.arm_subsystem.rotateWrist(0);
    // }
    
    //TJ Declared unDeprecated
    _robot.arm_subsystem.setWristSpeed(_robot.operatorStick.getRawAxis(1));//2
    _robot.arm_subsystem.setArmSpeed(-_robot.operatorStick.getRawAxis(5));//0

    
    /*if(_robot.arm_subsystem.getToggly()){
      wristy.SetTargetAngle(_robot.arm_subsystem.heights[1][2]);
    }
    else{
      wristy.SetTargetAngle(_robot.arm_subsystem.heights[1][6]);
    }
    */
    //Check if joystick is active
    /*if(Helpers.DeadbandJoystick( _robot.operatorStick.getRawAxis(1), _robot.robotMap) > 0)
    {
      double angleDifferencePercent = ((_robot.robotMap.maxWristAngle - wristy.getCurrentAngle()) * Helpers.DeadbandJoystick( _robot.operatorStick.getRawAxis(1), _robot.robotMap));
      wristy.SetTargetAngle(wristy.getCurrentAngle() + angleDifferencePercent);
    }
    else if(Helpers.DeadbandJoystick( _robot.operatorStick.getRawAxis(1), _robot.robotMap) < 0)
    {
      double angleDifferencePercent = ((_robot.robotMap.minWristAngle + wristy.getCurrentAngle()) * Helpers.DeadbandJoystick( _robot.operatorStick.getRawAxis(1), _robot.robotMap));
      wristy.SetTargetAngle(wristy.getCurrentAngle() + angleDifferencePercent);
    }
    else{
      wristy.SetTargetAngle(wristy.getTargetAngle());
    }*/



    //this will literally never do anything
    /*
    if(false){
      _robot.arm_subsystem.setWristSpeed(wristy.GetAnglePidOutput(Math.toDegrees(Math.atan2(_robot.wristnavX.getQuaternionY(), _robot.wristnavX.getQuaternionW()))));
    }
    */
    //_robot.arm_subsystem.setWristSpeed(wristy.GetAnglePidOutput(Math.toDegrees(Math.atan2(_robot.wristnavX.getQuaternionY(), _robot.wristnavX.getQuaternionW()))));
  }
  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    _robot.arm_subsystem.rotate(0);
    _robot.arm_subsystem.rotateWrist(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
