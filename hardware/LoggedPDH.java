// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VectorKit.hardware;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class LoggedPDH extends SubsystemBase {
  private final PowerDistribution kPDH;

  private double lt = 0, dt = 0;
  private double runningEffAhMatch = 0, runningEffAhTotal = 0;
  private double runningAMatch = 0, runningATotal = 0;
  private double runningAhMatch = 0, runningAhTotal = 0;
  private Map<mechs, Double> runningATotals = new HashMap<>();
  private Map<mechs, Double> runningAhTotals = new HashMap<>();
  // private double sumIk = 0.0, totalTime = 0.0;
  private double Iref = 0;
  private int matchNum = 0;
  private boolean matchStarted = false;
  private String logBase = "";

  public LoggedPDH() {
    this(1, ModuleType.kRev);
  }

  public LoggedPDH(int ID, ModuleType type) {
    kPDH = new PowerDistribution(ID, type);

    for (var mech : mechs.values()) {
      runningATotals.put(mech, 0.0);
      runningAhTotals.put(mech, 0.0);
    }

    // Set the match identifier based on the match type (if FMS is attached)
    if (DriverStation.isFMSAttached()) {
      // Get the match number if FMS is attached
      matchNum = DriverStation.getMatchNumber();

      String mt = "";
      switch (DriverStation.getMatchType()) {
        case Practice:
          mt = "Prac";
          break;
        case Qualification:
          mt = "Qual";
          break;
        case Elimination:
          mt = "Elim";
          break;
        default:
          mt = "Invalid";
          break;
      }

      logBase = String.format("PDH/%sMatch-%d/", mt, matchNum);
    }
  }

  // Using data collected from: https://www.revrobotics.com/content/docs/ES17-12_User_Guide.pdf
  private double getPeukertsExponent(double currentAmps) {
    double lA = Math.log(currentAmps);
    return 0.02 * lA * lA + 0.05 * lA + 1.08;
  }

  private double getEffectiveAh(double currentAmps) {
    double k = getPeukertsExponent(currentAmps);
    double dtHours = dt / 3600.0;

    // TODO: Dynamic Iref -- Requires extensive testing
    // sumIk += Math.pow(currentAmps, k) * dtHours;
    // totalTime += dtHours;
    // double Iref = Math.pow(sumIk / totalTime, 1.0 / k);

    return Math.pow(currentAmps, k) * dtHours / Math.pow(Iref, k - 1);
  }

  public LoggedPDH withIref(double Iref) {
    this.Iref = Iref;
    return this;
  }

  @Override
  public void periodic() {
    // Reset running match data on DS enable
    if (DriverStation.isEnabled() && !matchStarted) {
      runningEffAhMatch = 0;
      runningAMatch = 0;
      runningAhMatch = 0;
      if (!DriverStation.isFMSAttached()) matchNum += 1;
    }

    // Update the Delta Time
    double time = Timer.getFPGATimestamp();
    dt = time - lt;

    // Log per-channel amperage data
    double[] chLogs = kPDH.getAllCurrents();
    for (int i = 0; i < chLogs.length; i++) {
      double A = chLogs[i], Ah = A * dt / 3600.0;
      Logger.recordOutput(String.format("PDH/Amps/ch-%d", i), A);
      Logger.recordOutput(String.format("PDH/Amp Hours/ch-%d", i), Ah);
      switch (i) {
        case 0, 1, 3, 4, 15, 16, 17, 18, 19:
          runningATotals.put(mechs.Drivetrain, runningATotals.get(mechs.Drivetrain) + A);
          runningAhTotals.put(mechs.Drivetrain, runningAhTotals.get(mechs.Drivetrain) + Ah);
          break;
        case 7, 13:
          runningATotals.put(mechs.Intake, runningATotals.get(mechs.Intake) + A);
          runningAhTotals.put(mechs.Intake, runningAhTotals.get(mechs.Intake) + Ah);
          break;
        case 8:
          runningATotals.put(mechs.Transition, runningATotals.get(mechs.Transition) + A);
          runningAhTotals.put(mechs.Transition, runningAhTotals.get(mechs.Transition) + Ah);
          break;
        case 5, 6, 10, 11:
          runningATotals.put(mechs.Shooter, runningATotals.get(mechs.Shooter) + A);
          runningAhTotals.put(mechs.Shooter, runningAhTotals.get(mechs.Shooter) + Ah);
          break;
        case 14:
          runningATotals.put(mechs.FullSend, runningATotals.get(mechs.FullSend) + A);
          runningAhTotals.put(mechs.FullSend, runningAhTotals.get(mechs.FullSend) + Ah);
          break;
        case 20, 21, 22:
          runningATotals.put(mechs.Misc, runningATotals.get(mechs.Misc) + A);
          runningAhTotals.put(mechs.Misc, runningAhTotals.get(mechs.Misc) + Ah);
          break;
      }
    }

    // Log total amperage data
    double tA = kPDH.getTotalCurrent(), tAh = tA * dt / 3600.0, eAh = getEffectiveAh(tA);
    Logger.recordOutput("PDH/Amps", tA);
    Logger.recordOutput("PDH/Amp Hours", tAh);
    Logger.recordOutput("PDH/Eff Amp Hours", eAh);

    for (var total : runningATotals.entrySet())
      Logger.recordOutput(
          String.format("PDH/%s Amps", total.getKey().toString()), total.getValue());
    for (var total : runningAhTotals.entrySet())
      Logger.recordOutput(
          String.format("PDH/%s Amp Hours", total.getKey().toString()), total.getValue());

    // Update running totals
    runningAMatch += tA;
    runningAhMatch += tAh;

    runningATotal += tA;
    runningAhTotal += tAh;

    runningEffAhTotal += eAh;
    runningEffAhMatch += eAh;

    // Log running totals
    Logger.recordOutput("PDH/Running Totals/Amps", runningATotal);
    Logger.recordOutput("PDH/Running Totals/Amp Hours", runningAhTotal);
    Logger.recordOutput("PDH/Running Totals/Effective Ah", runningEffAhTotal);

    // Update per-match data on DS disable
    if (!DriverStation.isEnabled() && matchStarted) {
      if (!DriverStation.isFMSAttached()) logBase = String.format("PDH/Match-%d/", matchNum);

      Logger.recordOutput(String.format("%s/Amps", logBase), runningAMatch);
      Logger.recordOutput(String.format("%s/Amp Hours", logBase), runningAhMatch);
      Logger.recordOutput(String.format("%s/Effective Ah", logBase), runningEffAhMatch);
    }

    // Update helper variables
    matchStarted = DriverStation.isEnabled();
    lt = time;
  }

  private enum mechs {
    Drivetrain,
    Intake,
    Shooter,
    FullSend,
    Transition,
    Misc
  }
}
