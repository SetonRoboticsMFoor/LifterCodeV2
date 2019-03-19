/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//--------------------------------------------------------------------------------------------------
//
//
/**
This command is moves the motor forward to a setpoint
*/
//
//
//--------------------------------------------------------------------------------------------------

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class BothLiftersDownCom extends Command {
  private double frontLiftPotVoltage;
  private double rearLiftPotVoltage;

  public BothLiftersDownCom() {
    requires(Robot.lifterSub);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {

    frontLiftPotVoltage = Robot.lifterSub.getFrontLiftPotVoltage();
    rearLiftPotVoltage = Robot.lifterSub.getRearLiftPotVoltage();

    // Updates the SmartDashboard/ Shuffleboard with current values
    Robot.lifterSub.shuffleUpdate();
    // Continuously gets the value of the encoder
    Robot.lifterSub.getRearLiftPotVoltage();
    Robot.lifterSub.getFrontLiftPotVoltage();

    if ((frontLiftPotVoltage - rearLiftPotVoltage) > 30) {
      Robot.lifterSub.setFrontLiftPosition(1);
    }

    else if ((rearLiftPotVoltage - frontLiftPotVoltage) > 30) {
      Robot.lifterSub.setRearLiftPosition(1);
    }

    else {
      Robot.lifterSub.setFrontLiftPosition(1);
      Robot.lifterSub.setRearLiftPosition(1);
    }
    // --------------------------------------------------------------------------------------------------
    // The value below changes the setpoint for the command, change the value after
    // setPosition(change this value)
    // For this program, use a voltage between .5-4.5 (leave room for overshoot, you
    // don't want the
    // potentiometer to break)
    // --------------------------------------------------------------------------------------------------

  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {

    // Stops the motor when the command is finished, though technically it does not
    // run as the PID code
    // will keep running to keep the motor in place

    Robot.lifterSub.stopLiftMotors();
  }

  @Override
  protected void interrupted() {
  }
}
