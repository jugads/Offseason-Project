package frc.robot.subsystems.Bluetooth;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BluetoothSubsystem extends SubsystemBase {
    BluetoothIO io;
    BluetoothIO.BluetoothIOInputs inputs = new BluetoothIO.BluetoothIOInputs();
    public boolean hasCoral;
    public WantedState wantedState = WantedState.IDLE;
    public SystemState systemState = SystemState.IDLING;
    public int counter = 0;
    public int scoreCounter = 0;
    public BluetoothSubsystem(BluetoothIOSparkMax io) {
        this.io = io;
    }

    public enum WantedState {
        SCORING,
        SUCKING,
        STALLING,
        IDLE
    }

    public enum SystemState {
        HAS_CORAL,
        SEARCHING,
        READY_TO_SCORE,
        IDLING
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    @Override
    public void periodic() {
        io.refreshData();
        io.updateInputs(inputs);
        SmartDashboard.putString("Bluetooth/State", systemState.toString());
        SmartDashboard.putBoolean("Bluetooth/Has Coral", hasCoral);
        SmartDashboard.putNumber("Bluetooth/Score Count", scoreCounter);
        systemState = handleStateTransition();
        switch(systemState) {
            default:
            case IDLING:
                if (hasCoral) {
                    io.setMotor(0.05);
                }
                else {
                io.setMotor(0.0);
                }
                break;
            case SEARCHING:
                io.setMotor(0.8);
                scoreCounter = 0;
                break;
            case READY_TO_SCORE:
                io.setMotor(-0.8);
                scoreCounter++;
                break;
            case HAS_CORAL:
                io.setMotor(0.05);
                scoreCounter = 0;
                break;
        }
        if (inputs.current > 10) {
            counter++;
        }
        if (counter > 5) {
            hasCoral = true;
        }
        if (scoreCounter > 8) {
            hasCoral = false;
        }
        if (inputs.current < 10 && !hasCoral) {
            counter = 0;
        }
    }

    public SystemState handleStateTransition() {
        switch(wantedState) {
            case SCORING:
                return SystemState.READY_TO_SCORE;
            case STALLING:
                return SystemState.HAS_CORAL;
            case SUCKING:
                return SystemState.SEARCHING;
            case IDLE:
            default:
                return SystemState.IDLING;
        }
    }
    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    public WantedState getWantedState() {
        return wantedState;
    }
    public Command setWantedStateCommand(WantedState state) {
        return new InstantCommand(() -> setWantedState(state));
    }
    public void setHasCoral() {
        counter = 6;
        hasCoral = true;
    }
}
