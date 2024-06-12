package frc.robot.subsystems.MAXSwerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.MAXSwerve.DriveConstants.ModuleConstants;

public class ModuleIOSim implements ModuleIO {
    public static final double DRIVE_GEAR_RATIO = (45.0 * 22.0) / (14.0 * 15.0);
    public static final double TURN_GEAR_RATIO = 9424.0 / 203.0;
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNeoVortex(1), DRIVE_GEAR_RATIO, 0.02);
    private DCMotorSim turnSim = new DCMotorSim(DCMotor.getNeo550(1), TURN_GEAR_RATIO, 0.003);

    private SwerveModuleState optimizedState = new SwerveModuleState();

    private PIDController turnController = new PIDController(1, 0, 0);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 2.002); //2.002
    private PIDController driveController = new PIDController(0.04, 0, 0);

    private int n;

    public ModuleIOSim(int n, double chassisAngularOffset) {
        this.n = n;
        turnController.enableContinuousInput(0, 2 * Math.PI);
    }

    @Override
    public void resetEncoders() {
        driveSim.setState(0, 0);
        turnSim.setState(0, 0);
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(state,
                new Rotation2d(Drive.model.getTurnAngularPosition(n)));

        double driveOutput = feedforward.calculate(optimizedDesiredState.speedMetersPerSecond) + driveController.calculate(Drive.model.getDriveAngularVelocity(n) * ModuleConstants.kWheelDiameterMeters / 2, optimizedDesiredState.speedMetersPerSecond);
        double turnOutput = turnController.calculate(Drive.model.getTurnAngularPosition(n), optimizedDesiredState.angle.getRadians());
        
        if (DriverStation.isEnabled()) {
            Drive.model.setDriveVoltage(n, MathUtil.clamp(12 * driveOutput, -12, 12));
            Drive.model.setTurnVoltage(n, MathUtil.clamp(12 * turnOutput, -12, 12));
        }

        optimizedState = new SwerveModuleState(optimizedDesiredState.speedMetersPerSecond, optimizedDesiredState.angle);
    }

    public double turnWrapping(double angleRad){
        if(angleRad >= 0){
            return (angleRad % (2 * Math.PI));
        } else {
            angleRad = angleRad % (2 * Math.PI);
            return angleRad + (2 * Math.PI);
        }
    }

    @Override
    public SwerveModuleState getOptimizedState() {
        return optimizedState;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // driveSim.update(LOOP_PERIOD_SECS);
        // turnSim.update(LOOP_PERIOD_SECS);

        if (DriverStation.isDisabled()) {
            Drive.model.setTurnVoltage(n, 0);
            Drive.model.setDriveVoltage(n, 0);
        }

        inputs.drivePositionMeters = Drive.model.getDriveAngularPosition(n) * ModuleConstants.kWheelDiameterMeters / 2;
        inputs.driveVelocityMetersPerSec = Drive.model.getDriveAngularVelocity(n) * ModuleConstants.kWheelDiameterMeters / 2;
        inputs.turnAbsolutePosition = Rotation2d.fromRadians(Drive.model.getTurnAngularPosition(n));
        inputs.turnVelocityRadPerSec = Drive.model.getTurnAngularVelocity(n);
    }

}
