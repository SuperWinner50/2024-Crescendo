package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class IndexerIOReal implements IndexerIO {
    private CANSparkMax bottomIndexMotor = new CANSparkMax(IndexerConstants.leftCanID, MotorType.kBrushless);
    private CANSparkMax towerIndexMotor = new CANSparkMax(IndexerConstants.rightCanID, MotorType.kBrushless);
    private RelativeEncoder bottomEncoder = bottomIndexMotor.getEncoder();
    private RelativeEncoder towerEncoder = towerIndexMotor.getEncoder();
    private SparkLimitSwitch noteSensor = bottomIndexMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    private boolean spinBottom = true;

    public IndexerIOReal() {
        towerIndexMotor.restoreFactoryDefaults();
        bottomIndexMotor.restoreFactoryDefaults();
        
        noteSensor.enableLimitSwitch(false);

        towerIndexMotor.setSmartCurrentLimit(25);
        bottomIndexMotor.setSmartCurrentLimit(25);

        towerEncoder.setVelocityConversionFactor(IndexerConstants.kVelocityConversionFactor);
        bottomEncoder.setVelocityConversionFactor(IndexerConstants.kVelocityConversionFactor);

        bottomEncoder.setPosition(0);
        towerEncoder.setPosition(0);

        towerIndexMotor.setIdleMode(IdleMode.kBrake);
        bottomIndexMotor.setIdleMode(IdleMode.kCoast);

        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        towerIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        bottomIndexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

        towerIndexMotor.burnFlash();
        bottomIndexMotor.burnFlash();
    }

    @Override
    public void setIndexerSpeed(double speed) {
        if (spinBottom) {
            bottomIndexMotor.set(speed);
        }
        towerIndexMotor.set(speed);
    }

    @Override
    public void canSpinBottom(boolean spin) {
        spinBottom = spin;
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {    
        inputs.bottomMotorSpeed = bottomEncoder.getVelocity();
        inputs.bottomMotorCurrent = bottomIndexMotor.getOutputCurrent();
        inputs.bottomMotorVoltage = bottomIndexMotor.getAppliedOutput();

        inputs.towerMotorSpeed = towerEncoder.getVelocity();
        inputs.towerMotorCurrent = towerIndexMotor.getOutputCurrent();
        inputs.towerMotorVoltage = towerIndexMotor.getAppliedOutput();

        inputs.sensorActivated = noteSensor.isPressed();
    }
}