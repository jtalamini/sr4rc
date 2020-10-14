package it.units.erallab;

import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.erallab.hmsrobots.tasks.Task;
import org.dyn4j.dynamics.Settings;

public class ControllerOptimizationTask {

    static Task<?, ?> locomotion = new Locomotion(
            20,
            Locomotion.createTerrain("flat"),
            Lists.newArrayList(Locomotion.Metric.TRAVEL_X_RELATIVE_VELOCITY),
            new Settings()
    );

    static Task<?, ?> hiking = new Locomotion(
            20,
            new double[][]{{0.0D, 3.0D, 30.0D, 40.0D, 42D, 45D, 55.0D, 57.0D, 60.0D, 65.0D, 70D, 80D, 180D}, {100.0D, 0.0D, 0.0D, 5.0D, 4D, 5D, 5.0D, 7.0D, 5.0D, 5.0D, 2D, 4.0D, 2.0D}},
            5.0d,
            Lists.newArrayList(Locomotion.Metric.TRAVEL_X_RELATIVE_VELOCITY),
            new Settings()
    );

    static Task<?, ?> stairway = new Locomotion(
            20,
            new double[][]{{0, 5, 30, 30.1, 40, 40.1, 50, 50.1, 60, 60.1, 70, 70.1, 80, 80.1, 90D, 200D}, {100, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 6}}, //stairway
            5.0d,
            Lists.newArrayList(Locomotion.Metric.TRAVEL_X_RELATIVE_VELOCITY),
            new Settings()
    );

    static Task<?, ?> jump = new Jump(
            20,
            Jump.createTerrain("bowl"),
            25.0,
            Lists.newArrayList(Jump.Metric.CENTER_JUMP),
            new Settings()
    );
}
