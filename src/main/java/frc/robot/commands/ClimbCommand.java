/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class ClimbCommand extends Command {
  private static final double CLIMB_SPEED = 0.7;
  private static final double EXTEND_SPEED = 0.5;

  /**
   * Creates a new ClimbCommand.
   */
  public ClimbCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    requires(Robot.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Robot.oi.shooterController.getRawButton(1)) {
      SmartDashboard.putString("ClimbState", "CLIMB UP");
      Robot.climber.climb(CLIMB_SPEED);
    } else if(Robot.oi.shooterController.getRawButton(2)){
      SmartDashboard.putString("ClimbState", "LOWER");
      Robot.climber.climb(-CLIMB_SPEED);
    } else {
      SmartDashboard.putString("ClimbState", "STOP");
      Robot.climber.climb(0);
    }

    if(Robot.oi.shooterController.getRawButton(4)) {
      SmartDashboard.putString("ExtendState", "RETRACT");
      Robot.climber.extend(EXTEND_SPEED);
    } else if(Robot.oi.shooterController.getRawButton(3)){
      SmartDashboard.putString("ExtendState", "EXTEND");
      Robot.climber.extend(-EXTEND_SPEED);
    } else {
      SmartDashboard.putString("ExtendState", "STOP");
      Robot.climber.extend(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void interrupted() {
    Robot.climber.climb(0);
    Robot.climber.extend(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
