/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.DriveXY;
import frc.robot.commands.LoadBall;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.commands.ShootBall;

public class DriveToWallAndShoot extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DriveToWallAndShoot() {
    addSequential(new SetShooterSpeed(ManualShooterCommand.NEAR_SETPOINT));
    addSequential(new DriveXY(100, 0, 0, 0.5));
    addSequential(new DriveXY(120, 0, 0, 0.2));
    addSequential(new SetShooterSpeed(ManualShooterCommand.NEAR_SETPOINT));
    addSequential(new LoadBall());
    addSequential(new ShootBall());
    addSequential(new SetShooterSpeed(ManualShooterCommand.NEAR_SETPOINT));
    addSequential(new LoadBall());
    addSequential(new ShootBall());
    addSequential(new SetShooterSpeed(ManualShooterCommand.NEAR_SETPOINT));
    addSequential(new LoadBall());
    addSequential(new ShootBall());
    addSequential(new DriveXY(100, 20, 0, 0.3));
    addSequential(new DriveXY(100, 50, 0, 0.3));
  }
}
