package frc.robot.subsystems.AlgaeGripper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaeGripper.AlgaeGripperIO.AlgaeGripperInputs;


public class AlgaeGripperSubsystem extends SubsystemBase{
    AlgaeGripperIO io;
    AlgaeGripperIO.AlgaeGripperInputs inputs = new AlgaeGripperInputs();
    public boolean hasAlgae;
    public int counter;
    public WantedState wantedState = WantedState.IDLE;
    public SystemState systemState = SystemState.IDLING;
    public AlgaeGripperSubsystem(AlgaeGripperIO io) {
        this.io = io;
    }
    public enum WantedState {
        SCORING,
        SUCKING,
        STALLING,
        IDLE
    }

    public enum SystemState {
        HAS_ALGAE,
        SEARCHING,
        READY_TO_SCORE,
        IDLING
    }

    public boolean hasAlgae() {
        return hasAlgae;
    }

    @Override
    public void periodic() {
        io.refreshData();
        io.updateInputs(inputs);
        SmartDashboard.putString("AlgaeGripper/State", systemState.toString());
        SmartDashboard.putBoolean("AlgaeGripper/Has Algae", hasAlgae);
        systemState = handleStateTransition();
        switch(systemState) {
            default:
            case IDLING:
                if (hasAlgae) {
                    io.setMotor(0.05);
                }
                else {
                io.setMotor(0.0);
                }
                break;
            case SEARCHING:
                io.setMotor(0.8);
                break;
            case READY_TO_SCORE:
                io.setMotor(-1.0);
                break;
            case HAS_ALGAE:
                io.setMotor(.05);
                break;
        }
        if (inputs.distance < 0.08) {
            counter++;
        }
        if (counter > 5) {
            hasAlgae = true;
        }
        if (inputs.seeingAlgae == false || inputs.distance > 0.08) {
            hasAlgae = false;
        }
    }

     public SystemState handleStateTransition() {
        switch(wantedState) {
            case IDLE:
            default:
                return SystemState.IDLING;
            case SCORING:
                return SystemState.READY_TO_SCORE;
            case STALLING:
                return SystemState.HAS_ALGAE;
            case SUCKING:
                return SystemState.SEARCHING;
        }
    }
    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    public WantedState getWantedState() {
        return wantedState;
    }
    public Command setWantedStateCommand(WantedState scoring) {
        return new InstantCommand(() -> setWantedState(scoring));
    }
}
