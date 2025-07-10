// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaeGripper.AlgaeGripperSubsystem;
import frc.robot.subsystems.Bluetooth.BluetoothSubsystem;
import static edu.wpi.first.units.Units.*;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  AddressableLED leds;
  AddressableLEDBuffer buffer;
  BluetoothSubsystem bluetooth;
  AlgaeGripperSubsystem algae;
  Timer timer = new Timer();

  public LEDs(AddressableLED leds, AddressableLEDBuffer buffer, BluetoothSubsystem bluetooth, AlgaeGripperSubsystem algae) {
    this.leds = leds;
    this.buffer = buffer;
    this.bluetooth = bluetooth;
    this.algae = algae;
    timer.start();
    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled()) {
      cycleColor(Color.kDarkMagenta);
    }
    else if (bluetooth.hasCoral()) {
      setAll(Color.kWhite);
    }
    else if (algae.hasAlgae()) {
      setAll(Color.kGreen);
    }
    else {
      setAll(Color.kBlack);
    }

  }
  public void cycleColor(Color color) {
    LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, color);
    LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(99));

    pattern.applyTo(buffer);
    leds.setData(buffer);
  }

  public void setAll(Color color) {
    LEDPattern pattern = LEDPattern.solid(color);
    pattern.applyTo(buffer);
    leds.setData(buffer);
  }

  public void flash(Color color) {
    if ((int)(timer.get() * 8) % 2 == 0) {
      
      setAll(color);
    } else {
      setAll(Color.kBlack);
    }
  }
}
