/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class CompressorSubsystem extends Subsystem {
    private final Compressor compressor;
    public CompressorSubsystem(Robot robot){
        compressor = new Compressor();
        compressor.setClosedLoopControl(true);
    }

    public void checkPressure() {
        if(!compressor.getPressureSwitchValue()) {
            compressor.stop();
        } else {
            compressor.start();
        }
    }

    public boolean pressure() {
        return compressor.getPressureSwitchValue();
    }

    @Override
    protected void initDefaultCommand() {
    }
}