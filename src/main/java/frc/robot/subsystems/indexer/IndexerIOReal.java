package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IndexerIOReal implements IndexerIO {
    private CANSparkMax leftIndexMotor = new CANSparkMax(IndexerConstants.leftCanID, MotorType.kBrushless);
    private CANSparkMax rightIndexMotor = new CANSparkMax(IndexerConstants.rightCanID, MotorType.kBrushless);
    private RelativeEncoder leftEncoder = leftIndexMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightIndexMotor.getEncoder();
    private SparkLimitSwitch noteSensor = leftIndexMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    public IndexerIOReal() {
        rightIndexMotor.restoreFactoryDefaults();
        leftIndexMotor.restoreFactoryDefaults();
        
        noteSensor.enableLimitSwitch(false);

        rightIndexMotor.setSmartCurrentLimit(25);
        leftIndexMotor.setSmartCurrentLimit(25);

        rightEncoder.setVelocityConversionFactor(IndexerConstants.kVelocityConversionFactor);
        leftEncoder.setVelocityConversionFactor(IndexerConstants.kVelocityConversionFactor);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        rightIndexMotor.setIdleMode(IdleMode.kCoast);
        leftIndexMotor.setIdleMode(IdleMode.kCoast);

        rightIndexMotor.burnFlash();
        leftIndexMotor.burnFlash();
    }

    @Override
    public void setIndexerSpeed(double speed) {
        leftIndexMotor.set(speed);
        rightIndexMotor.set(speed);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {    
        inputs.leftMotorSpeed = leftEncoder.getVelocity();
        inputs.leftMotorCurrent = leftIndexMotor.getOutputCurrent();
        inputs.leftMotorVoltage = leftIndexMotor.getAppliedOutput();

        inputs.leftMotorSpeed = leftEncoder.getVelocity();
        inputs.leftMotorCurrent = leftIndexMotor.getOutputCurrent();
        inputs.leftMotorVoltage = leftIndexMotor.getAppliedOutput();

        inputs.sensorActivated = noteSensor.isPressed();
    }
}
