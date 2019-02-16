/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.commands.HatchEjector;
import frc.robot.commands.HatchEjectorToggle;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShiftGear;
import frc.robot.commands.ManualDriveDirection;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  Joystick opStick = new Joystick(1);
  Joystick DrvStick = new Joystick(0);
  Button intakeButton = new JoystickButton(opStick, 6);
  Button ejectButton = new JoystickButton(opStick, 5);
  Button hatchIn = new JoystickButton(opStick, 3);
  Button hatchOut = new JoystickButton(opStick, 4);
  // Button hatchEjectButton = new JoystickButton(stick, 1);
  POVButton forwards = new POVButton(DrvStick,0);
  POVButton leftwards = new POVButton(DrvStick,270); 
  POVButton backwards = new POVButton(DrvStick,180);
  POVButton rightwards = new POVButton(DrvStick,90);
  POVButton northeast = new POVButton(DrvStick,45);
  POVButton southeast = new POVButton(DrvStick,135);
  POVButton southwest = new POVButton(DrvStick,225);
  POVButton northwest = new POVButton(DrvStick,315);
  Button vision = new JoystickButton(DrvStick, 2);
  // Button ShiftGearButton = new JoystickButton(stick, 2);
  public OI(Robot robot){

    hatchIn.whenPressed(new HatchEjector(robot, true));
    hatchOut.whenPressed(new HatchEjector(robot, false));

    intakeButton.whileHeld(new IntakeBall(robot, true));
    ejectButton.whileHeld(new IntakeBall(robot, false));
    // hatchEjectButton.whenPressed(new HatchEjector(robot, !robot.arm_subsystem.getHatchEjectorValue())); 
    forwards.whileHeld(new ManualDriveDirection(robot, 0));
    backwards.whileHeld(new ManualDriveDirection(robot, 180));
    leftwards.whileHeld(new ManualDriveDirection(robot, 270));
    rightwards.whileHeld(new ManualDriveDirection(robot, 90));
    northeast.whileHeld(new ManualDriveDirection(robot, 45));
    southeast.whileHeld(new ManualDriveDirection(robot, 135));
    southwest.whileHeld(new ManualDriveDirection(robot, 225));
    northwest.whileHeld(new ManualDriveDirection(robot, 315));
    //ShiftGearButton.whenPressed(new ShiftGear(robot, true));
    //ShiftGearButton.whenReleased(new ShiftGear(robot, false));
    // hatchEjectButton.whenPressed(new HatchEjectorToggle(new HatchEjector(robot,true),new HatchEjector(robot,false),robot));
    
  }
  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  
}
