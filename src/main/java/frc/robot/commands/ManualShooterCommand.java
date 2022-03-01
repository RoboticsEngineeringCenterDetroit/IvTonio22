/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;

public class ManualShooterCommand extends Command {
  /**
   * Creates a new ManualShooterCommand.
   */

  static final double INCREMENT = 25.0;
  static final double NEAR_SETPOINT = 5500.0;
  static final double FAR_SETPOINT = 5300.0;

  double rpmSetpoint = 0.0;

  public ManualShooterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    requires(Robot.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rpmSetpoint = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    int pov = Robot.oi.shooterController.getPOV();
    if(pov == 0) {
      rpmSetpoint += INCREMENT;
    } else if(pov == 180) {
      rpmSetpoint -= INCREMENT;
    } else if(pov == 90) {
      rpmSetpoint = NEAR_SETPOINT;
    } else if(pov == 270) {
      rpmSetpoint = FAR_SETPOINT;
    }

    rpmSetpoint = MathUtil.clamp(rpmSetpoint, 0.0, Shooter.MAX_RPM
    );

    double rpm = Robot.shooter.getRpm();
    double sendValue = MathUtil.clamp(rpmSetpoint, rpm - 500, rpm + 500);
    Robot.shooter.setMotorRPM(sendValue);
    SmartDashboard.putNumber("Shooter Setpoint", rpmSetpoint);

    double feedspeed;
    if(Robot.shooter.ballReady()) {
      feedspeed = Robot.oi.shooterController.getRawAxis(3);
    } else {
      feedspeed = Robot.oi.shooterController.getRawAxis(2);
    }
    
    feedspeed = Utilities.deadband(feedspeed, 0.10);
    Robot.shooter.setFeedSpeed(feedspeed);
  }

  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
