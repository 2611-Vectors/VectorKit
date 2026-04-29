// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VectorKit.tuners;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class PidTuner {
  private final LoggedNetworkNumber loggedP, loggedI, loggedD, loggedS, loggedV;
  private double kP, kI, kD, kS, kV;

  public PidTuner(String folder) {
    loggedP = new LoggedNetworkNumber(String.format("%s/P", folder), 0.0);
    loggedI = new LoggedNetworkNumber(String.format("%s/I", folder), 0.0);
    loggedD = new LoggedNetworkNumber(String.format("%s/D", folder), 0.0);
    loggedS = new LoggedNetworkNumber(String.format("%s/S", folder), 0.0);
    loggedV = new LoggedNetworkNumber(String.format("%s/V", folder), 0.0);
  }

  public PidTuner(String folder, double P, double I, double D) {
    loggedP = new LoggedNetworkNumber(String.format("%s/P", folder), P);
    loggedI = new LoggedNetworkNumber(String.format("%s/I", folder), I);
    loggedD = new LoggedNetworkNumber(String.format("%s/D", folder), D);
    loggedS = new LoggedNetworkNumber(String.format("%s/S", folder), 0.0);
    loggedV = new LoggedNetworkNumber(String.format("%s/V", folder), 0.0);
  }

  public PidTuner(String folder, double P, double I, double D, double S, double V) {
    loggedP = new LoggedNetworkNumber(String.format("%s/P", folder), P);
    loggedI = new LoggedNetworkNumber(String.format("%s/I", folder), I);
    loggedD = new LoggedNetworkNumber(String.format("%s/D", folder), D);
    loggedS = new LoggedNetworkNumber(String.format("%s/S", folder), S);
    loggedV = new LoggedNetworkNumber(String.format("%s/V", folder), V);
  }

  public boolean updated() {
    boolean updated = false;
    if (kP != loggedP.get()) {
      kP = loggedP.get();
      updated = true;
    }

    if (kI != loggedI.get()) {
      kI = loggedI.get();
      updated = true;
    }

    if (kD != loggedD.get()) {
      kD = loggedD.get();
      updated = true;
    }

    if (kS != loggedS.get()) {
      kS = loggedS.get();
      updated = true;
    }

    if (kV != loggedV.get()) {
      kV = loggedV.get();
      updated = true;
    }

    return updated;
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
}
