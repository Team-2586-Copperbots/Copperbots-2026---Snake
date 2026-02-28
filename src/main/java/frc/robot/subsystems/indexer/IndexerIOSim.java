package frc.robot.subsystems.indexer;

import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class IndexerIOSim implements IndexerIO {
    private double towerSpeed = 0;
    private double spindexerSpeed = 0;

    public SimulatedMotorController.GenericMotorController towerMotor = new GenericMotorController(DCMotor.getFalcon500(1));

    public IndexerIOSim() {

    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.spindexerSpeed = spindexerSpeed;
        inputs.towerSpeed = towerSpeed;
    }

    @Override
    public void setTowerSpeed(double output) {
        towerSpeed = output;
    }

    @Override
    public void setSpindexerSpeed(double output) {
        spindexerSpeed = output;
    }
}
