/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.commands.ManualShooterCommand;
import edu.wpi.first.wpiutil.math.*;

public class Shooter extends Subsystem {

  private Double rpmSetpoint;

  private final int SHOOTER_MOTOR_CAN_ID = 15;
  private final int FEED_MOTOR_CAN_ID = 8;

  public static final double DEFAULT_RPM = 0.0;
  public static final double MAX_RPM = 6500.0;
  public static final double MAX_VEL = 22000.0;

  /**
   * Creates a new shooter.
   */

  TalonFX shooterMotor;
  CANSparkMax feedMotor;
  DigitalInput feedTriggerSwitch;

  public Shooter() {
    
    shooterMotor = new TalonFX(SHOOTER_MOTOR_CAN_ID);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.config_kF(0, 0.048, 0);
    shooterMotor.config_kP(0, 0.010, 0);


    feedMotor = new CANSparkMax(FEED_MOTOR_CAN_ID, MotorType.kBrushless);
    feedMotor.setIdleMode(IdleMode.kBrake);
    feedMotor.setSmartCurrentLimit(40);

    rpmSetpoint = DEFAULT_RPM;

    feedTriggerSwitch = new DigitalInput(0);
  }

  public Double getRpm() {
    int countsPerHundredMs = shooterMotor.getSelectedSensorVelocity();
    Double rpm = countsPerHundredMs * 60000.0 / (100 * 2048);

    return rpm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Ball Ready", ballReady());
    SmartDashboard.putNumber("Shooter RPM", getRpm());
    //SmartDashboard.putNumber("Shooter VEL", shooterMotor.getSelectedSensorVelocity());
  }

  public boolean ballReady() {
    return !feedTriggerSwitch.get();
  }

  public void setMotorRPM(double rpm) {

    double velocity = (rpm * 100.0 * 2048.0 )/ 60000.0;

    shooterMotor.set(ControlMode.Velocity, velocity);
    //System.out.println("shooter velocity = " + velocity);
  }

  public void setMotorPercent(double value) {
    shooterMotor.set(ControlMode.PercentOutput, value);
  }

  public void setMotorVelocity(double vel) {
    shooterMotor.set(ControlMode.Velocity, vel);
  }

  public void setFeedSpeed(double speed) {
    feedMotor.set(speed);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualShooterCommand());
  }
}
