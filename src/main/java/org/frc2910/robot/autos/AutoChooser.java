package org.frc2910.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc2910.robot.RobotContainer;
import org.frc2910.robot.config.Constants;

import java.util.stream.Collectors;
import java.util.stream.Stream;
import java.util.*;

/**
 * A {@link edu.wpi.first.wpilibj.smartdashboard.SendableChooser} for selecting an autonomous program.
 * <br/><br/>
 * <p>
 * How to add a new autonomous program.
 *     <ol>
 *         <li>
 *             Add a new value to {@link Auto}<br/>
 *             The name of the value should use screaming snake case
 *             </li>
 *             <li>
 *                 Add a new method in {@link AutoFactory} that returns a {@link edu.wpi.first.wpilibj2.command.Command}.
 *             </li>
 *             <li>
 *                 Add a new {@link AutoProgram} to {@link AutoChooser#buildAutoPrograms}.
 *             </li>
 *             <li>
 *                 Implement the autonomous program factory method.
 *             </li>
 *             <li>
 *                 Test, test, and test some more.
 *             </li>
 *         </ol>
 * </p>
 */
public class AutoChooser extends SendableChooser<Auto> {

    private static List<AutoProgram> buildAutoPrograms() {
        final List<AutoProgram> programs = new ArrayList<>(List.of(
                new AutoProgram(Auto.IDLE, "IDLE", AutoFactory::createIdleCommand),
                new AutoProgram(Auto.TEST, "TEST", AutoFactory::createTestAuto)));

        if (Constants.SystemCharacterizationConstants.ENABLE_SYSID_PROGRAMS) {
            programs.addAll(List.of(
                    new AutoProgram(Auto.SYSID_TRANSLATION, "SysId / Translation", AutoFactory::sysIdTranslation),
                    new AutoProgram(Auto.SYSID_ROTATION, "SysId / Rotation", AutoFactory::sysIdRotation),
                    new AutoProgram(Auto.SYSID_STEER, "SysId / Steer", AutoFactory::sysIdSteer)));
        } else {
            programs.addAll(List.of(
                    new AutoProgram(
                            Auto.LEFT_NEUTRAL_ZONE, "LEFT_NEUTRAL_ZONE", AutoFactory::createLeftNeutralZoneAuto),
                    new AutoProgram(
                            Auto.RIGHT_NEUTRAL_ZONE, "RIGHT_NEUTRAL_ZONE", AutoFactory::createRightNeutralZoneAuto),
                    new AutoProgram(
                            Auto.LEFT_NEUTRAL_ZONE_TO_DEPOT,
                            "LEFT_NEUTRAL_ZONE_TO_DEPOT",
                            AutoFactory::createLeftNeutralZoneToDepotAuto),
                    new AutoProgram(Auto.DEPOT, "DEPOT", AutoFactory::createDepotAuto)));
        }

        return programs;
    }

    private static final List<AutoProgram> AUTO_PROGRAMS = buildAutoPrograms();

    /**
     * Create a new <code>AutoChooser</code>
     *
     * @param robotContainer A {@link RobotContainer}
     * @return A new <code>AutoChooser</code> populated with the programs defined in the private field {@link AutoChooser#AUTO_PROGRAMS}.
     */
    public static AutoChooser create(final RobotContainer robotContainer) {
        var autoFactories = Stream.of(DriverStation.Alliance.values())
                .map(alliance -> Map.entry(alliance, new AutoFactory(alliance, robotContainer)))
                .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));
        var programs = AUTO_PROGRAMS.stream()
                .map(program -> Map.entry(program.getAuto(), program))
                .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));

        var autoChooser = new AutoChooser(programs, autoFactories);

        AUTO_PROGRAMS.forEach(program -> {
            if (program.getAuto() == Auto.IDLE) {
                autoChooser.setDefaultOption(program.getLabel(), program.getAuto());
            } else {
                autoChooser.addOption(program.getLabel(), program.getAuto());
            }
        });

        autoChooser.reset(null);

        Shuffleboard.getTab(Constants.operatorDashboardName)
                .addString("Selected Auto", () -> autoChooser.getSelected().name())
                .withPosition(12, 3)
                .withSize(6, 2)
                .withWidget(BuiltInWidgets.kTextView);

        return autoChooser;
    }

    /**
     * Update the <code>AutoChooser</code><br/><br/>
     *
     * <p>
     * The commands for the selected autonomous program are loaded and cached (if not already present)
     * and the selected program is sent to the {@link edu.wpi.first.wpilibj.shuffleboard.Shuffleboard} under the key <code>Auto/Selected</code>.
     * </p>
     */
    public void update() {
        var selected = getSelected();

        Stream.of(DriverStation.Alliance.values()).forEach(alliance -> commandCache
                .get(alliance)
                .computeIfAbsent(
                        selected, auto -> Pair.of(loadStartingPose(alliance, auto), loadCommand(alliance, auto))));
    }

    /**
     * Reset the caches behind this <code>AutoChooser</code>
     *
     * @param key Optional {@link edu.wpi.first.networktables.NetworkTable} key.
     *            If provided the selected value of the entry at this location will also be reset.
     */
    public void reset(final String key) {
        Stream.of(DriverStation.Alliance.values())
                .forEach(alliance -> commandCache.get(alliance).clear());

        if (key != null) {
            var table = NetworkTableInstance.getDefault().getTable(key);
            table.putValue("selected", NetworkTableValue.makeString("%s".formatted(Auto.IDLE)));
        }
    }

    /**
     * Get the {@link Command} for the selected autonomous program, if available.
     *
     * @return The {@link Command} for the selected autonomous program as an {@link Optional} if available,
     * otherwise {@link Optional#empty}.
     */
    public Optional<Command> getSelectedCommand() {
        var selected = getSelected();

        return DriverStation.getAlliance()
                .map(alliance -> commandCache.get(alliance).get(selected).getSecond());
    }

    public Optional<Pose2d> getStartingPose() {
        var selected = getSelected();

        return DriverStation.getAlliance()
                .map(alliance -> commandCache.get(alliance).get(selected).getFirst());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.publishConstString("selected", "%s".formatted(Auto.IDLE));
    }

    private Command loadCommand(final DriverStation.Alliance alliance, final Auto auto) {
        var program = programs.get(auto);

        System.out.printf("Loading command %s/%s\n", alliance, auto);

        return program.getCommand(autoFactories.get(alliance));
    }

    private Pose2d loadStartingPose(final DriverStation.Alliance alliance, final Auto auto) {
        var program = programs.get(auto);

        System.out.printf("Loading starting pose %s/%s\n", alliance, auto);

        return program.getStartingPose(autoFactories.get(alliance));
    }

    private final Map<Auto, AutoProgram> programs;

    private final Map<DriverStation.Alliance, Map<Auto, Pair<Pose2d, Command>>> commandCache;

    private final Map<DriverStation.Alliance, AutoFactory> autoFactories;

    private AutoChooser(
            final Map<Auto, AutoProgram> programs, final Map<DriverStation.Alliance, AutoFactory> autoFactories) {
        this.programs = programs;
        this.autoFactories = autoFactories;
        commandCache = Stream.of(DriverStation.Alliance.values())
                .map(alliance -> Map.entry(alliance, new HashMap<Auto, Pair<Pose2d, Command>>()))
                .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));
    }
}
