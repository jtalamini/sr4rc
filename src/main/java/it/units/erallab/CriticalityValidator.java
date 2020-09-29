package it.units.erallab;

import it.units.erallab.hmsrobots.core.controllers.Controller;
import it.units.erallab.hmsrobots.core.controllers.TimeFunctions;
import it.units.erallab.hmsrobots.core.objects.ControllableVoxel;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.Voxel;
import it.units.erallab.hmsrobots.util.Grid;
import org.apache.commons.lang3.SerializationUtils;
import org.dyn4j.dynamics.Settings;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.Objects;
import java.util.stream.IntStream;

public class CriticalityValidator {

    public static void main(String[] args) {

        //String serializedBest = "H4sIAAAAAAAAAOxTv28TMRR+vbZqaNXSJAUxwEQYmxsqIaEMaZS7lEjXFuUiBO1Q3MRJXPl+4PO11w6V+APYKYKNgYGVAQkWliImaEf+AySkLuz4XZJrAAlBBVIGnmT7+fOzv+/Zzy++wHggIMdkPnSZDPJUEM7JZr7jBMLb9BQSSsbzS4I13567erF0/fClBqNVGOmotmOBJgMJs9YW2SY6RuoWC2Qh8gFgEps6PIuL3WNKQpBdjIgeHF05OCRPR2GkCmMB26PxlubOGPa+r7bd+IWmhido3tvcog01KXuuFB5GcHrbiyj/+mTj3bXqzcsaaAakHBJVPNGgFky1cFymsuM1JSxZTOoxgd4j0BMCHQn0HoH+E0GucnpQIRIw/7tS491vJqJnJ/dWX2swbUCaCEpqRDJvmUQG5ZIoyS3BGgpx12COM4fJWACJoQonbQPGHBIEBmRwKLntkBNhEMdnbnsN0giWlVwWJPExZjGXJnEGZBGzWZNa1G3LTqzBgClBA8lkiFyGer5k3YCJwBe4M/EqFmS6nt0grZbHm8pV5ZAdKAfTDR2bYkUUP0Bsi3e740jxpOv0x8QpHi/EVnx1gPZ4MfU9vjjXnWPhntZWjylnU8EIZ3uYj3tLeNHu+MTxp+36+4+qHlQRUE4d6sr6rk8lpLtSOXHbepmrCymsQ6oXoTJJrw+sI4HKY1tA8Y/eO2f/eEe9fCEDoKpnJs4BKeIcBhf9UPQ1nAYUnj/8PL9wpD/SMEJFavv3YR9mJEzbVcPcMO/UzdpKyUrQsrlSr5WsjXJt1bb7178vwDzzDxv8AIN6kfG8hJRRteullbIZIDD+D14ez53EbjbEPo23gE4Wuwt9Ipxc8vs2RGKGQcp/DcOjYXiqwv8LFn0DAAD//wMATgdfyVYIAAA=";

        final ControllableVoxel softMaterial = new ControllableVoxel(
                Voxel.SIDE_LENGTH,
                Voxel.MASS_SIDE_LENGTH_RATIO,
                5d, // low frequency
                Voxel.SPRING_D,
                Voxel.MASS_LINEAR_DAMPING,
                Voxel.MASS_ANGULAR_DAMPING,
                Voxel.FRICTION,
                Voxel.RESTITUTION,
                Voxel.MASS,
                Voxel.LIMIT_CONTRACTION_FLAG,
                Voxel.MASS_COLLISION_FLAG,
                Voxel.AREA_RATIO_MAX_DELTA,
                EnumSet.of(Voxel.SpringScaffolding.SIDE_EXTERNAL, Voxel.SpringScaffolding.CENTRAL_CROSS), // scaffolding partially enabled
                ControllableVoxel.MAX_FORCE,
                ControllableVoxel.ForceMethod.DISTANCE
        );

        Grid<ControllableVoxel> best = Grid.create(10, 10, (x,y) -> SerializationUtils.clone(softMaterial)); //Utils.safelyDeserialize(serializedBest, Grid.class);

        int bodySize = (int) best.values().stream().filter(Objects::nonNull).count();

        int[] avalanchesSpatialExtension = new int[bodySize + 1];
        int[] avalanchesTemporalExtension = new int[1000];

        double finalT = 30;
        double pulseDuration = 0.4;
        // 0.0002
        double avalancheThreshold = 0.002;
        int binSize = 5;

        // task
        CriticalityEvaluator criticalityEvaluator = new CriticalityEvaluator(
                finalT, // task duration
                new Settings(), // default settings for the physics engine
                avalancheThreshold
        );

        // a pulse controller is applied on each voxel
        IntStream.range(0, (int) Math.pow(best.getW(), 2d)).forEach(i -> {
            Controller<ControllableVoxel> pulseController = new TimeFunctions(Grid.create(best.getW(), best.getW(), (x, y) -> (Double t) -> {
                if (x * best.getW() + y == i) {
                    if (t < pulseDuration/2) {
                        return 1.0;
                    } else if (t < pulseDuration) {
                        return -1.0;
                    }
                }
                return 0.0;
            }));
            List<Double> metrics = criticalityEvaluator.apply(new Robot<>(pulseController, SerializationUtils.clone(best)));

            if (metrics.get(0).intValue() > 0) {
                avalanchesSpatialExtension[metrics.get(0).intValue()] += 1;
            }
            if (metrics.get(1).intValue() > 0) {
                avalanchesTemporalExtension[(metrics.get(1).intValue()) / binSize] += 1;
            }
        });
        // exit condition
        int spatialSizeNumber = (int) Arrays.stream(avalanchesSpatialExtension).filter(frequency -> frequency > 0).count();
        int temporalSizeNumber = (int) Arrays.stream(avalanchesTemporalExtension).filter(frequency -> frequency > 0).count();
    }
}
