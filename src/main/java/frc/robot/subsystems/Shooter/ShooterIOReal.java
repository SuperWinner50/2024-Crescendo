package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.KrakenLogger;

public class ShooterIOReal implements ShooterIO {
    private TalonFX topShooterMotor = new TalonFX(ShooterConstants.rightCanID);
    private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.leftCanID);

    private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    KrakenLogger topMotorLogger = new KrakenLogger(topShooterMotor, ShooterConstants.rightCanID);
    KrakenLogger bottomMotorLogger = new KrakenLogger(bottomShooterMotor, ShooterConstants.leftCanID);

    public ShooterIOReal(){
        talonFXConfig.Slot0.kP = ShooterConstants.kP;
        talonFXConfig.Slot0.kI = 0;
        talonFXConfig.Slot0.kD = 0;
        talonFXConfig.Slot0.kS = 0;
        talonFXConfig.Slot0.kV = 0.12;

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = 60;

        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // talonFXConfig.MotionMagic.MotionMagicAcceleration = 100;
        // talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = 10;

        talonFXConfig.Audio.BeepOnBoot = true;

        bottomShooterMotor.clearStickyFaults();
        topShooterMotor.clearStickyFaults();

        StatusCode response = bottomShooterMotor.getConfigurator().apply(talonFXConfig);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + bottomShooterMotor.getDeviceID()
                            + " failed config with error "
                            + response.toString());
        }

        response = topShooterMotor.getConfigurator().apply(talonFXConfig);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + topShooterMotor.getDeviceID()
                            + " failed config with error "
                            + response.toString());
        }
    }

    @Override
    public void runVoltageTop(double volts) {
        topShooterMotor.setVoltage(volts);
    }

    @Override
    public void runVoltageBottom(double volts) {
        bottomShooterMotor.setVoltage(volts);
    }

    @Override
    public void runShooterMotorsRPM(double topSpeed, double bottomSpeed) {
        // set velocity to certain rps, add 0.5 V to overcome gravity
        topShooterMotor.setControl(m_request.withVelocity(topSpeed / 60));
        bottomShooterMotor.setControl(m_request.withVelocity(bottomSpeed / 60));

        // bottomShooterMotor.set(MathUtil.clamp(bottomSpeed, -1, 1));
        // topShooterMotor.set(MathUtil.clamp(topSpeed, -1, 1));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        topMotorLogger.update();
        bottomMotorLogger.update();
        
        inputs.bottomShooterMotorAppliedVolts = bottomShooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.bottomShooterMotorVelocityRPM = bottomShooterMotor.getVelocity().getValueAsDouble() * 60.0; // RPS to RPM
        inputs.bottomShooterCurrent = bottomShooterMotor.getStatorCurrent().getValueAsDouble();
        inputs.topShooterMotorAppliedVolts = topShooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.topShooterMotorVelocityRPM = topShooterMotor.getVelocity().getValueAsDouble() * 60.0; // RPS to RPM
        inputs.topShooterCurrent = topShooterMotor.getStatorCurrent().getValueAsDouble();
    }
}