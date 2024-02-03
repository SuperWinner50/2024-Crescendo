package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.util.LimelightHelpers;

public class LimelightLookAtSpeaker {

    private static PIDController rotationPID = new PIDController(0.013, 0, 0.00125); // Change values

    public static Command lookAtSpeaker(Drive drive) {
        return Commands.run(
                () -> {
                    double x = LimelightHelpers.getTX("limelight");
                    double output = rotationPID.calculate(x, 0);
                    Logger.recordOutput("Vision/LLOutput", output);
                    Logger.recordOutput("Vision/LLX", x);
                    drive.drive(
                            -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftY(),
                                    OIConstants.kDriveDeadband),
                            -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftX(),
                                    OIConstants.kDriveDeadband),
                            MathUtil.clamp(output, -1, 1),
                            true, true);
                },
                drive);
    }

}
