/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */

/* Open Source Software - may be modified and shared by FRC teams. The code */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project. */

/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.TurnPid;
import frc.robot.WristPid;
import frc.robot.subsystems.*;

public class MoveToHeight extends Command {

  private ArmSubsystem arm;
  private Robot robot;
  private int position;
  private double turnpower;
  private WristPid level;
  private TurnPid armPid;


  public MoveToHeight(Robot r, int pos) {
    robot = r;
    requires(robot.arm_subsystem);
    arm = robot.arm_subsystem;
    position = pos;
    level = new WristPid(robot);
    armPid = new TurnPid(0.05, 0, 0, 0.01, 0.02, 0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(true){
      int pos = position + (arm.getToggly() && position != 0 ? 4 : 0);
      //arm.moveToHeightPreset(pos);
      level.SetTargetAngle(arm.heights[1][position]);
      turnpower = -level.GetAnglePidOutput(robot.arm_subsystem.getWistAngle());
      armPid.SetTargetAngle((double)(arm.heights[0][pos] * 360.0 / (4096.0 * 25)) - 30);
      arm.moveToHeightWrist(turnpower);
      arm.rotate(-armPid.GetAnglePidOutput(arm.getArmEncoderAngle()));
    }
    else
    {

      double armPower = 0;
      double armAngle = arm.getArmEncoderAngle();
      double wristAngle = robot.arm_subsystem.getWistAngle();
      int pos = position + (arm.getToggly() && position != 0 ? 4 : 0);
      //arm.moveToHeightPreset(pos);
      level.SetTargetAngle(arm.heights[1][position]);
      turnpower = -level.GetAnglePidOutput(wristAngle);
      armPid.SetTargetAngle((double)(arm.heights[0][pos] * 360.0 / (4096.0 * 25)) - 30);
      armPower = -armPid.GetAnglePidOutput(armAngle);

      float constraint = 56;
      float armLength = 39;
      float wristLength = 19;
      int threshold = 2;

      double checkForOut = armLength * Math.cos(Math.toRadians(armAngle + 90)) + wristLength * Math.cos(Math.toRadians(wristAngle - 45));
      double outArmD = -armLength * armPower * Math.sin(Math.toRadians(armAngle + 90));
      double outWristD = -wristLength * turnpower * Math.sin(Math.toRadians(wristAngle * 2 - 45));
      if(checkForOut > constraint - threshold && outArmD + outWristD > 0)
      {
        if(outArmD < 0 && outWristD > 0)
          turnpower = 0;
        if(outArmD > 0 && outWristD < 0)
          armPower = 0;
      }
      arm.moveToHeightWrist(turnpower);
      arm.rotate(armPower);
    }
  }

  // Make this return true when this Command no longer sneeds to run execute()
  @Override
  protected boolean isFinished() {
    // Keep holding the button in case the belt slips
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
