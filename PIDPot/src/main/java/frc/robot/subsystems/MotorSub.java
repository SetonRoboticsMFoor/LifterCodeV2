/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//--------------------------------------------------------------------------------------------------
//
//
/**
This is a working PID subsystem for a Talon SRX using an encoder
*/
//
//
//--------------------------------------------------------------------------------------------------



package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.MotorStopCom;

public class MotorSub extends Subsystem {

  // Initialize all of the objects and values for the
  // subsystem-----------------------------------------------
  private WPI_TalonSRX myMotor;
  private AnalogInput herderArmPot;
  private PIDController myPid;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  // The constructor for the
  // subsystem---------------------------------------------------------------------------
  public MotorSub() {

    // Instatiate the motor and encoder objects and assign values for the subsystem
    myMotor = new WPI_TalonSRX(RobotMap.MOTOR_CH);
    herderArmPot = new AnalogInput(RobotMap.HERDER_ARM_POT_CH);

    // Instantiate the PID controller oobject and enable it
    // kP is the P value, kI is the I value, kD is the D value, myEncoder would be
    // the name of your sensor
    // myMotor is the output motor,
    // PIDController(Pvalue, Ivalue, Dvalue, sensor that is being used, output
    // motor)
    myPid = new PIDController(kP, kI, kD, herderArmPot, myMotor);
    myPid.enable();

    //*** */ Change your PID values here to tune*** ///

    // Proportoinal (P) value shows the proportion of current error (distance from current position to setpoint)
    // This affects the speed towards the target and a high value will overshoot the target.
    // Start with a low value (.01) and increase until the motor moves in the correct direction and 
    // slightly undershoots the target value
    // Move this value up by small increments like .1
    kP = .4;

    // Integral (I) value is proportional to the accumulated error (the total error over time)
    // After tuning P value, slightly raise the I value until you see slight oscillation (moving
    // back and forth around the target)
    // Increase this value by VERY small increments like .01

    kI = 0;
    // The dertivative value (D) is the current change in error (how fast we are approaching the setpoint)
    // Move this value up to dampen (stop) the oscillation
    // Increase this value by VERY small increments like .01
    kD = 0;


    // These are used for velocity control, not position
    kIz = 0;
    kFF = 0;

    // Set the max and min output for the motor- start slow and then go faster
    // At full motor output and zero I/D you will see oscilation as the motor
    // settles
    // At slow speeds (like .1) the motor will not oscillate as much but will take
    // longer to get to the desired position
    // Start these values low at like .2 and increase by small values like .1
    kMaxOutput = .3;
    kMinOutput = -.3;

    
    //*********************************************************************** */
    //*********************************************************************** */
    //*********************************************************************** */
    //DO NOT CHANGE ANYTHING BELOW THESE LINES
    //*********************************************************************** */
    //*********************************************************************** */
    //*********************************************************************** */

    
    // Assign the given PID values to the PID controller object- *****DO NOT CHANGE*****
    myPid.setP(kP);
    myPid.setI(kI);
    myPid.setD(kD);

    // These two are not used for a non CAN controller
    // myPid.setIZone(kIz);
    // myPid.setFF(kFF);

    // Take the max and min output values and assign them to the PID controller
    // This is used to control the motor output - *****DO NOT CHANGE*****
    myPid.setOutputRange(kMinOutput, kMaxOutput);

    // Creates the initial values for the SmartDashboard - *****DO NOT CHANGE*****
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Voltage", 0);
  }

  // This method gets the encoder value and writes it to the SmartDashBoard - *****DO NOT CHANGE*****
  public void getVoltage() {

    SmartDashboard.putNumber("Herder Arm Potentiometer Position", herderArmPot.getVoltage());

  }
  // End of
  // constructor-----------------------------------------------------------------------------------

  // This method is for testing the motor and is not used
  public void motorForward() {
    myMotor.set(.25);
  }

  // The method to stop the motor
  public void motorStop() {
    myMotor.set(0);
  }

  // Update the information on the SmartDashboard as they
  // update--------------------------------------------------------------------------------------
  // - *****DO NOT CHANGE*****

  public void shuffleUpdate() {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double voltage = SmartDashboard.getNumber("Set Voltage", 0);

    if ((p != kP)) {
      myPid.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      myPid.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      myPid.setD(d);
      kD = d;
    }
    // if((iz != kIz)) { myPid.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { myPid.setFF(ff); kFF = ff; }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      myPid.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }
  }

  // Create the method that gets the setpoint
  // ----------------------------------------------------------
  // ************************************************************ */
  // ** DO NOT CHANGE THE VALUES HERE- CHANGE THEM IN THE COMMANDS */
  // ************************************************************ */
  public void setPosition(double voltage) {
    myPid.setSetpoint(voltage);

    SmartDashboard.putNumber("SetPoint", voltage);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MotorStopCom());
  }
}
