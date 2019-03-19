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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.MotorStopCom;

public class LifterSub extends Subsystem {

  // Initialize all of the objects and values for the
  // subsystem-----------------------------------------------
  private WPI_VictorSPX frontLifterMotor;
  private WPI_VictorSPX rearLifterMotor;
  private WPI_VictorSPX liftDriveMotor;

  private Encoder frontLiftEncoder;
  private Encoder rearLiftEncoder;

  private AnalogInput frontLiftPot;
  private AnalogInput rearLiftPot;

  private PIDController frontLiftPid;
  private PIDController rearLiftPid;

  public double fKP, fKI, fKD, fKIz, fKFF, fKMaxOutput, fKMinOutput;
  public double rKP, rKI, rKD, rKIz, rKFF, rKMaxOutput, rKMinOutput;

  // The constructor for the
  // subsystem---------------------------------------------------------------------------
  public LifterSub() {

    // Instatiate the motor and encoder objects and assign values for the subsystem
    frontLifterMotor = new WPI_VictorSPX(RobotMap.FRONT_LIFT_MOTOR_CH);
    rearLifterMotor = new WPI_VictorSPX(RobotMap.REAR_LIFT_MOTOR_CH);
    liftDriveMotor = new WPI_VictorSPX(RobotMap.LIFT_DRIVE_MOTOR_CH);

    frontLiftEncoder = new Encoder(RobotMap.FRONT_LIFT_ENCODER_CH_1, RobotMap.FRONT_LIFT_ENCODER_CH_2);
    rearLiftEncoder = new Encoder(RobotMap.REAR_LIFT_ENCODER_CH_1, RobotMap.REAR_LIFT_ENCODER_CH_2);

    frontLiftPot = new AnalogInput(RobotMap.FRONT_LIFT_POT_CH);
    rearLiftPot = new AnalogInput(RobotMap.REAR_LIFT_POT_CH);

    // Instantiate the PID controller oobject and enable it
    // kP is the P value, kI is the I value, kD is the D value, myEncoder would be
    // the name of your sensor
    // myMotor is the output motor,
    // PIDController(Pvalue, Ivalue, Dvalue, sensor that is being used, output
    // motor)
    frontLiftPid = new PIDController(fKP, fKI, fKD, frontLiftPot, frontLifterMotor);
    frontLiftPid.enable();

    fKP = .4;
    fKI = 0;
    fKD = 0;
    fKIz = 0;
    fKFF = 0;

    fKMaxOutput = .3;

    rearLiftPid = new PIDController(rKP, rKI, rKD, rearLiftPot, rearLifterMotor);
    rearLiftPid.enable();

    rKP = .4;
    rKI = 0;
    rKD = 0;
    rKIz = 0;
    rKFF = 0;

    rKMinOutput = -.3;

    frontLiftPid.setP(fKP);
    frontLiftPid.setI(fKI);
    frontLiftPid.setD(fKD);

    // These two are not used for a non CAN controller
    // myPid.setIZone(kIz);
    // myPid.setFF(kFF);

    // Take the max and min output values and assign them to the PID controller
    // This is used to control the motor output - *****DO NOT CHANGE*****
    frontLiftPid.setOutputRange(fKMinOutput, fKMaxOutput);
    rearLiftPid.setOutputRange(rKMinOutput, rKMaxOutput);

    // Creates the initial values for the SmartDashboard - *****DO NOT CHANGE*****
    SmartDashboard.putNumber("P Gain", fKP);
    SmartDashboard.putNumber("I Gain", fKI);
    SmartDashboard.putNumber("D Gain", fKD);
    SmartDashboard.putNumber("I Zone", fKIz);
    SmartDashboard.putNumber("Feed Forward", fKFF);
    SmartDashboard.putNumber("Max Output", fKMaxOutput);
    SmartDashboard.putNumber("Min Output", fKMinOutput);
    SmartDashboard.putNumber("Set Voltage", 0);

    SmartDashboard.putNumber("P Gain", rKP);
    SmartDashboard.putNumber("I Gain", rKI);
    SmartDashboard.putNumber("D Gain", rKD);
    SmartDashboard.putNumber("I Zone", rKIz);
    SmartDashboard.putNumber("Feed Forward", rKFF);
    SmartDashboard.putNumber("Max Output", rKMaxOutput);
    SmartDashboard.putNumber("Min Output", rKMinOutput);
    SmartDashboard.putNumber("Set Voltage", 0);
  }

  // This method gets the encoder value and writes it to the SmartDashBoard -
  // *****DO NOT CHANGE*****
  public double getFrontLiftPotVoltage() {
    SmartDashboard.putNumber("Front Lifter Potentiometer Position", frontLiftPot.getVoltage());
    return frontLiftPot.getVoltage();
  }

  public double getRearLiftPotVoltage() {
    SmartDashboard.putNumber("Rear Lifter Potentiometer Position", rearLiftPot.getVoltage());
    return frontLiftPot.getVoltage();
  }
  // End of
  // constructor-----------------------------------------------------------------------------------

  // Update the information on the SmartDashboard as they
  // update--------------------------------------------------------------------------------------
  // - *****DO NOT CHANGE*****

  public void shuffleUpdate() {

    // Update Shuffleboard values for the front
    // lifter______________________________________
    double fP = SmartDashboard.getNumber("Front Lift P Gain", 0);
    double fI = SmartDashboard.getNumber("Front Lift I Gain", 0);
    double fD = SmartDashboard.getNumber("Front Lift D Gain", 0);
    double fIz = SmartDashboard.getNumber("Front Lift I Zone", 0);
    double fF = SmartDashboard.getNumber("Front Lift Feed Forward", 0);
    double fMax = SmartDashboard.getNumber("Front Lift Max Output", 0);
    double fMin = SmartDashboard.getNumber("Front Lift Min Output", 0);
    double fVoltage = SmartDashboard.getNumber("Front Lift Set Voltage", 0);

    if ((fP != fKP)) {
      frontLiftPid.setP(fP);
      fKP = fP;
    }
    if ((fI != fKI)) {
      frontLiftPid.setI(fI);
      fKI = fI;
    }
    if ((fD != fKD)) {
      frontLiftPid.setD(fD);
      fKD = fD;
    }
    // if((iz != kIz)) { myPid.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { myPid.setFF(ff); kFF = ff; }
    if ((fMax != fKMaxOutput) || (fMin != fKMinOutput)) {
      frontLiftPid.setOutputRange(fMin, fMax);
      fKMinOutput = fMin;
      fKMaxOutput = fMax;

      // Update Shuffleboard values for the rear
      // lifter______________________________________

      double rP = SmartDashboard.getNumber("Rear Lift P Gain", 0);
      double rI = SmartDashboard.getNumber("Rear Lift I Gain", 0);
      double rD = SmartDashboard.getNumber("Rear Lift D Gain", 0);
      double rIz = SmartDashboard.getNumber("Rear Lift I Zone", 0);
      double rF = SmartDashboard.getNumber("Rear Lift Feed Forward", 0);
      double rMax = SmartDashboard.getNumber("Rear Lift Max Output", 0);
      double rMin = SmartDashboard.getNumber("Rear Lift Min Output", 0);
      double rVoltage = SmartDashboard.getNumber("Rear Lift Set Voltage", 0);

      if ((rP != rKP)) {
        rearLiftPid.setP(rP);
        rKP = rP;
      }
      if ((rI != rKI)) {
        rearLiftPid.setI(rI);
        rKI = rI;
      }
      if ((rD != rKD)) {
        rearLiftPid.setD(rD);
        rKD = rD;
      }
      // if((iz != kIz)) { myPid.setIZone(iz); kIz = iz; }
      // if((ff != kFF)) { myPid.setFF(ff); kFF = ff; }
      if ((rMax != rKMaxOutput) || (rMin != rKMinOutput)) {
        rearLiftPid.setOutputRange(rMin, rMax);
        rKMinOutput = rMin;
        rKMaxOutput = rMax;
      }
    }
  }

  // Create the method that gets the setpoint
  // ----------------------------------------------------------
  // ************************************************************ */
  // ** DO NOT CHANGE THE VALUES HERE- CHANGE THEM IN THE COMMANDS */
  // ************************************************************ */

  public double getFrontLiftEncoder() {
    return frontLiftEncoder.getDistance();
  }

  public double getRearLiftEncoder() {
    return rearLiftEncoder.getDistance();
  }


  public void setFrontLiftPosition(double frontVoltage) {
    frontLiftPid.setSetpoint(frontVoltage);

    SmartDashboard.putNumber("SetPoint", frontVoltage);

  }

  public void setRearLiftPosition(double rearVoltage) {
    frontLiftPid.setSetpoint(rearVoltage);

    SmartDashboard.putNumber("SetPoint", rearVoltage);

  }

  public void stopLiftMotors() {
    frontLifterMotor.set(0);
    rearLifterMotor.set(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MotorStopCom());
  }
}
