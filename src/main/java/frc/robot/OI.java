/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.BothLiftersDownCom;
import frc.robot.commands.FrontLifterUpCom;
import frc.robot.commands.LiftDriveForwardCom;
import frc.robot.commands.LiftDriveReverseCom;
import frc.robot.commands.RearLifterUpCom;

public class OI {

  // Initialize the objects for the joystick and joystick buttons
  private Joystick elevatorStick;
  private JoystickButton retractFrontLiftButton;
  private JoystickButton retractRearLiftButton;
  private JoystickButton extendBothButton;
  private JoystickButton liftDriveForwardButton;
  private JoystickButton liftDriveReverseButton;

  // Create the constructor that runs when the OI class is instantiated
  // Instatiate the joysticks and buttons, then bind them to the commands
  public OI() {

    elevatorStick = new Joystick(RobotMap.ELEVATOR_STICK_CH);

    extendBothButton = new JoystickButton(elevatorStick, RobotMap.EXTEND_BOTH_BUTTON_CH);
    extendBothButton.whenPressed(new BothLiftersDownCom());

    retractFrontLiftButton = new JoystickButton(elevatorStick, RobotMap.FRONT_LIFT_RETRACT_BUTTON_CH);
    retractFrontLiftButton.whenPressed(new FrontLifterUpCom());

    retractRearLiftButton = new JoystickButton(elevatorStick, RobotMap.REAR_LIFT_RETRACT_BUTTON_CH);
    retractRearLiftButton.whenPressed(new RearLifterUpCom());

    liftDriveForwardButton = new JoystickButton(elevatorStick, RobotMap.LIFT_DRIVE_FORWARD_CH);
    liftDriveForwardButton.whileHeld(new LiftDriveForwardCom());

    liftDriveReverseButton = new JoystickButton(elevatorStick, RobotMap.LIFT_DRIVE_REVERSE_CH);
    liftDriveReverseButton.whileHeld(new LiftDriveReverseCom());
  }
}
