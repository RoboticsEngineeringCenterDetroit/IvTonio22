/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends Command {

  public static final double ANGLE_TOLERANCE = 5.0;

  double target;

  public TurnToAngle(double targetAngle) {
    requires(Robot.driveTrain);
    target = targetAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.driveHeading(new Translation2d(), target);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(delta()) < ANGLE_TOLERANCE;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.stop();
  }

  private double delta() {
    return DriveTrain.angleDelta(Robot.driveTrain.getAngle(), target);
  }
}
