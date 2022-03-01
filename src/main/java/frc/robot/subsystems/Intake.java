/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;


public class Intake extends Subsystem {
  /**
   * Creates a new Intake.
   */

  public static final int LEFT_INTAKE_CAN_ID  = 12;
  public static final int RIGHT_INTAKE_CAN_ID = 13;
  
  VictorSPX leftIntakeMotor, rightIntakeMotor;

  public Intake() {
    leftIntakeMotor = new VictorSPX(LEFT_INTAKE_CAN_ID);
    rightIntakeMotor = new VictorSPX(RIGHT_INTAKE_CAN_ID);
    leftIntakeMotor.setNeutralMode(NeutralMode.Coast);
    rightIntakeMotor.setNeutralMode(NeutralMode.Coast);
  }


  public void setIntakeSpeed(double speed) {
    speed = Utilities.deadband(speed, 0.10);
    leftIntakeMotor.set(ControlMode.PercentOutput, speed);
    rightIntakeMotor.set(ControlMode.PercentOutput, -speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //TODO move this dumb control code into a proper command
    double inSpeed = Robot.oi.shooterController.getRawAxis(1);
    setIntakeSpeed(inSpeed);
  }

  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub    
  }
}
