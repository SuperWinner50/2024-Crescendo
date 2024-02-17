package frc.robot.subsystems.indexer;

import java.beans.Statement;

import org.littletonrobotics.junction.Logger;

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

    // public void isStoring()

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        double averageCurrent = this.averageCurrent();
        currents.addLast(Double.valueOf(inputs.leftMotorCurrent));

        if(inputs.sensorActivated && StateManager.getInstance().state == State.GroundIntake){
            StateManager.getInstance().setState(State.Stow);
        }

        if(override){
            io.setIndexerSpeed(IndexerConstants.overrideSpeed);
        } else if (shouldSpin || StateManager.getInstance().state == State.GroundIntake){
            io.setIndexerSpeed(indexerSpeed);
        } else {
            io.setIndexerSpeed(0);
        }

        if(averageCurrent > IndexerConstants.currentThreshold){
            currentStoring = true;
        } else {
            currentStoring = false;
        }

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