// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VectorKit.tuners;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class TunablePidController {
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);
  private final PIDController pid = new PIDController(0, 0, 0);

  private final LoggedNetworkNumber loggedP, loggedI, loggedD, loggedS, loggedV, loggedTargetVel;
  private double kP, kI, kD, kS, kV, targetVel;

  public TunablePidController(String folder) {
    loggedP = new LoggedNetworkNumber(String.format("%s/P", folder), 0.0);
    loggedI = new LoggedNetworkNumber(String.format("%s/I", folder), 0.0);
    loggedD = new LoggedNetworkNumber(String.format("%s/D", folder), 0.0);
    loggedS = new LoggedNetworkNumber(String.format("%s/S", folder), 0.0);
    loggedV = new LoggedNetworkNumber(String.format("%s/V", folder), 0.0);
    loggedTargetVel = new LoggedNetworkNumber(String.format("%s/Target Velocity", folder), 0.0);
  }

  public TunablePidController(String folder, double P, double I, double D) {
    loggedP = new LoggedNetworkNumber(String.format("%s/P", folder), P);
    loggedI = new LoggedNetworkNumber(String.format("%s/I", folder), I);
    loggedD = new LoggedNetworkNumber(String.format("%s/D", folder), D);
    loggedS = new LoggedNetworkNumber(String.format("%s/S", folder), 0.0);
    loggedV = new LoggedNetworkNumber(String.format("%s/V", folder), 0.0);
    loggedTargetVel = new LoggedNetworkNumber(String.format("%s/Target Velocity", folder), 0.0);
  }

  public TunablePidController(
      String folder, double P, double I, double D, double S, double V, double targetVel) {
    loggedP = new LoggedNetworkNumber(String.format("%s/P", folder), P);
    loggedI = new LoggedNetworkNumber(String.format("%s/I", folder), I);
    loggedD = new LoggedNetworkNumber(String.format("%s/D", folder), D);
    loggedS = new LoggedNetworkNumber(String.format("%s/S", folder), S);
    loggedV = new LoggedNetworkNumber(String.format("%s/V", folder), V);
    loggedTargetVel =
        new LoggedNetworkNumber(String.format("%s/Target Velocity", folder), targetVel);
  }

  public void update() {
    if (kP != loggedP.get()) {
      kP = loggedP.get();
      pid.setP(kP);
    }

    if (kI != loggedI.get()) {
      kI = loggedI.get();
      pid.setI(kI);
    }

    if (kD != loggedD.get()) {
      kD = loggedD.get();
      pid.setD(kD);
    }

    if (kS != loggedS.get()) {
      kS = loggedS.get();
      ff.setKs(kS);
    }

    if (kV != loggedV.get()) {
      kV = loggedV.get();
      ff.setKv(kV);
    }

    if (targetVel != loggedTargetVel.get()) targetVel = loggedTargetVel.get();
  }

  public double getP() {
    return kP;
  }

  public double getI() {
    return kI;
  }

  public double getD() {
    return kD;
  }

  public double getS() {
    return kS;
  }

  public double getV() {
    return kV;
  }

  public double calculate(double measurement, double setpoint) {
    update();
    return pid.calculate(measurement, setpoint) + ff.calculate(targetVel);
  }
}
