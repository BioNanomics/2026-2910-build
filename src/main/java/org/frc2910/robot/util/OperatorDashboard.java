package org.frc2910.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import java.util.Map;

public class OperatorDashboard {
    ShuffleboardTab dashboard;
    private final SimpleWidget shooterPercentAdder;
    private final SimpleWidget passPercentAdded;

    public OperatorDashboard() {
        dashboard = Shuffleboard.getTab("Operator Dashboard");
        shooterPercentAdder = dashboard
                .add("RPM Percent Adder", 0.0)
                .withSize(5, 3)
                .withPosition(1, 1)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -10.0, "max", 50.0));
        passPercentAdded = dashboard
                .add("Pass RPM Percent Adder", 0.0)
                .withSize(5, 3)
                .withPosition(1, 4)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -10.0, "max", 50.0));
    }

    public double getRPMMultiplier() {
        return (1.0 + (shooterPercentAdder.getEntry().getDouble(0.0) / 100.0));
    }

    public double getRPMPassMultiplier() {
        return (1.0 + (passPercentAdded.getEntry().getDouble(0.0) / 100.0));
    }
}
