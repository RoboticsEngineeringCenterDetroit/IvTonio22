package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frcteam2910.common.robot.Utilities;

public class DriveCommand extends Command {

    public DriveCommand() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void execute() {
        double forward = -Robot.oi.driveController.getRawAxis(1);
        forward = Utilities.deadband(forward);
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = -Robot.oi.driveController.getRawAxis(0);
        strafe = Utilities.deadband(strafe);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation = -Robot.oi.driveController.getRawAxis(4);
        rotation = Utilities.deadband(rotation, 0.040);
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

        boolean fieldOrientedFlag;

        fieldOrientedFlag = !Robot.oi.driveController.getRawButton(6);

        if(Robot.oi.driveController.getRawButtonPressed(5)) {
            Robot.driveTrain.resetYaw();
        }

        Robot.driveTrain.drive(new Translation2d(forward, strafe), rotation, fieldOrientedFlag);

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
