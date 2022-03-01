/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public class DriveXY extends Command {

  public static final double TARGET_TOLERANCE = 3.0;
  public static final double RAMP_UP_TIME = 2.0;
  public static final double RAMP_DOWN_DISTANCE = 50;

  Pose2d target;
  Transform2d delta;
  long startTimeMicroSeconds;
  double speedScale;

  public Transform2d getDelta() {
    return target.minus(Robot.driveTrain.getPose());
  }

  public double getDistanceToTarget() {
    return getDelta().getTranslation().getNorm();
  }

  public DriveXY(double x, double y, double angleDegrees, double speed) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
    speedScale = speed;
    target = new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(angleDegrees));
  }

  

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTimeMicroSeconds = RobotController.getFPGATime();
    System.out.println("Starting Drive command:");
    System.out.println("from:" + Robot.driveTrain.getPose().toString());
    System.out.println("to:" + target.toString());
  }

  public double getElapsedTime() {
    return (RobotController.getFPGATime()  - startTimeMicroSeconds) / 1000000.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Translation2d translation = getDelta().getTranslation();
    translation = translation.div(translation.getNorm());

    double scale = speedScale;
    double dx = getDistanceToTarget();
    double time = getElapsedTime();
    double heading = target.getRotation().getDegrees();

    if(dx < RAMP_DOWN_DISTANCE) {
      scale = Math.min(speedScale, dx / RAMP_DOWN_DISTANCE);
    }

    if(time < RAMP_UP_TIME)
    {
      scale = Math.min(time / RAMP_UP_TIME, scale);
    }

    translation = translation.times(scale);

    //System.out.println("driving: " + translation.toString() + " heading: " + heading);
    //Robot.driveTrain.driveHeading(translation, heading);
    Robot.driveTrain.drive(translation, 0.0, true);
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(getDistanceToTarget()) < TARGET_TOLERANCE;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
    System.out.println("finished move in " + getElapsedTime() + "seconds");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.stop();
    System.out.println("interrupted move after" + getElapsedTime() + "seconds");
  }
}
