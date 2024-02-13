package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double topShooterMotorVelocityRPM = 0.0;
        public double topShooterMotorAppliedVolts = 0.0;
        public double bottomShooterMotorVelocityRPM = 0.0;
        public double bottomShooterMotorAppliedVolts = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void runShooterMotors(double topSpeed, double bottomSpeed) {}
}
