// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VectorKit.hardware;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class LinearActuator extends SubsystemBase {
    // https://wcproducts.com/products/wcp-0408

    private final Servo m_actuator;

    public LinearActuator(int channel, ActuatorModel model) {
        m_actuator = new Servo(channel);
        switch (model) {
            case WCP_0408:
                m_actuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
                break;
        }
    }

    public enum ActuatorModel {
        WCP_0408
    }
}
