// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */

  public VictorSPX feeder = new VictorSPX(3);

  public Feeder(boolean isInverted) {
    feeder.setInverted(isInverted);
    feeder.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runForward(){
    feeder.set(ControlMode.PercentOutput, 0.6);
  }
  public void runBackwards(){
    feeder.set(ControlMode.PercentOutput, -0.6);
  }
  public void stop(){
    feeder.set(ControlMode.PercentOutput, 0.0);
  }
}
