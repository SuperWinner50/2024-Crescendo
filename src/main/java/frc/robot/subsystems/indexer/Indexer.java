package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateManager;
import frc.robot.StateManager.State;

public class Indexer extends SubsystemBase {
    
    private IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged(); 
    private double indexerSpeed = 0.0;
    private boolean shouldSpin = false;
    private boolean override;


    private boolean currentStoring = false;
    private CircularBuffer<Double> currents = new CircularBuffer<>(IndexerConstants.bufferSize);

    private static Indexer instance;

    public static Indexer getInstance(){
        return instance;
    }

    public static Indexer initialize(IndexerIO IndexIo){
        if(instance == null){
            instance = new Indexer(IndexIo);
        }
        return instance;
    }

    private Indexer(IndexerIO IndexIo) {
        io = IndexIo;
        io.updateInputs(inputs);

        for(int i = 0; i < IndexerConstants.bufferSize; i++){
            currents.addLast(Double.valueOf(0));
        }
    }

    public void setIndexerSpeed(double motorValue) {
        indexerSpeed = motorValue;
    }

    public void setSpin(boolean on){
        shouldSpin = on;
    }

    public void reverse() {
        indexerSpeed *= -1;
    }

    public void setOverride(boolean on) {
       override = on;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        double averageCurrent = this.averageCurrent();
        currents.addLast(Double.valueOf(io.getLeftMotorCurrent()));

        if(override){
            io.setIndexerSpeed(IndexerConstants.overrideSpeed);
        } else if(StateManager.getInstance().state == State.GroundIntake || shouldSpin) {
            io.setIndexerSpeed(indexerSpeed);
        } else {
            io.setIndexerSpeed(0);
        }

        if(averageCurrent > IndexerConstants.currentThreshold){
            currentStoring = true;
        } else {
            currentStoring = false;
        }
        
        Logger.recordOutput("Indexer/Left Motor Current", io.getLeftMotorCurrent());
        Logger.recordOutput("Indexer/Left Motor Velocity", io.getLeftMotorVelocity());
        Logger.recordOutput("Indexer/Right Motor Current", io.getRightMotorCurrent());
        Logger.recordOutput("Indexer/Right Motor Velocity", io.getRightMotorVelocity());
        Logger.recordOutput("Indexer/Average current", averageCurrent);
        Logger.recordOutput("Indexer/Storing (based on current)", currentStoring);
    }

    private double averageCurrent(){
        double sum = 0;
        for(int i = 0; i < IndexerConstants.bufferSize; i++){
            sum += currents.get(i);
        }
        return (sum / IndexerConstants.bufferSize);
    }
}