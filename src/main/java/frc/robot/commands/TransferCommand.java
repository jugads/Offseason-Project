// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.kElevTran;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Bluetooth.BluetoothSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.WantedState;
import frc.robot.subsystems.Hopper.HopperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TransferCommand extends SequentialCommandGroup {
  /** Creates a new TransferCommand. */
  ElevatorSubsystem elevator;
  ArmSubsystem arm;
  HopperSubsystem hopper;
  BluetoothSubsystem bluetooth;
  public TransferCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HopperSubsystem hopperSubsystem, BluetoothSubsystem bluetoothSubsystem) {
    this.elevator = elevatorSubsystem;
    this.arm = armSubsystem;
    this.hopper = hopperSubsystem;
    this.bluetooth = bluetoothSubsystem;
    addCommands(
      elevator.setSetpointCommand(kElevTran),
      elevator.setWantedStateCommand(WantedState.GO_TO_SETPOINT),
      new WaitUntilCommand(() -> elevator.atPosition()),
      arm.setWantedStateCommand(ArmSubsystem.WantedState.GO_TO_TRANSFER),
      new WaitUntilCommand(() -> arm.getPosition() < -0.21),
      arm.setWantedStateCommand(ArmSubsystem.WantedState.HOLD),
      hopper.setWantedStateCommand(HopperSubsystem.WantedState.FEEDING),
      bluetooth.setWantedStateCommand(BluetoothSubsystem.WantedState.SUCKING),
      new WaitUntilCommand(() -> bluetooth.hasCoral),
      bluetooth.setWantedStateCommand(BluetoothSubsystem.WantedState.IDLE),
      elevator.setSetpointCommand(kElevTran+0.09),
      arm.setWantedStateCommand(ArmSubsystem.WantedState.GO_TO_REGULAR_SCORE)
    );
    addRequirements(elevator, arm, bluetooth, hopper);
  }
}
