/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.MotorForwardCom;
import frc.robot.commands.MotorReverseCom;

public class OI {

  // Initialize the objects for the joystick and joystick buttons
  private Joystick driveStick;
  private JoystickButton forwardButton;
  private JoystickButton reverseButton;

  // Create the constructor that runs when the OI class is instantiated
  // Instatiate the joysticks and buttons, then bind them to the commands
  public OI() {
    driveStick = new Joystick(RobotMap.STICK_CH);
    forwardButton = new JoystickButton(driveStick, RobotMap.FORWARD_BUTTON_CH);
    forwardButton.whenPressed(new MotorForwardCom());

    reverseButton = new JoystickButton(driveStick, RobotMap.REVERSE_BUTTON_CH);
    reverseButton.whenPressed(new MotorReverseCom());
  }
}
