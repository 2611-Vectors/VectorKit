// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VectorKit.hardware;

import edu.wpi.first.wpilibj.Servo;

/** Add your docs here. */
public class WCP_0408 extends Servo {
  // https://wcproducts.com/products/wcp-0408

  public WCP_0408(int channel) {
    super(channel);
    super.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }
}
