/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ShootBall extends Command {

  int counter = 0;                 //used for tracking time. Excute runs at 20ms I believe so 1 second = 50 loops (I think)

  public ShootBall() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    counter = 0;
    System.out.println("Shooting RPM = " + Robot.shooter.getRpm());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    counter++;                              //increment counter

    if (counter > 0) {                       //Can change from 0 to add a delay before you actually start feader to allow shooter wheel to spin up
      Robot.shooter.setFeedSpeed(1.0);
    }
    else {
      Robot.shooter.setFeedSpeed(0.0);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(counter>150) {   //this should make it shoot for 3 seconds if my math is correct, then the command will end
      return true;
    } 
    else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.shooter.setFeedSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.shooter.setFeedSpeed(0.0);
  }
}
