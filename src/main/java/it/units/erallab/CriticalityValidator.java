package it.units.erallab;

import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.core.controllers.Controller;
import it.units.erallab.hmsrobots.core.controllers.TimeFunctions;
import it.units.erallab.hmsrobots.core.objects.ControllableVoxel;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.viewers.GraphicsDrawer;
import it.units.erallab.hmsrobots.viewers.GridEpisodeRunner;
import it.units.erallab.hmsrobots.viewers.GridOnlineViewer;
import org.apache.commons.lang3.SerializationUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.dyn4j.dynamics.Settings;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.stream.IntStream;

import static it.units.malelab.jgea.core.util.Args.d;

public class CriticalityValidator {

    public static void main(String[] args) {

        String serializedBest = "H4sIAAAAAAAAAOxTv28TMRR+vbZqaNXSJAUxwEQYmxsqIaEMaZS7lEjXFuUiBO1Q3MRJXPl+4PO11w6V+APYKYKNgYGVAQkWliImaEf+AySkLuz4XZJrAAlBBVIGnmT7+fOzv+/Zzy++wHggIMdkPnSZDPJUEM7JZr7jBMLb9BQSSsbzS4I13567erF0/fClBqNVGOmotmOBJgMJs9YW2SY6RuoWC2Qh8gFgEps6PIuL3WNKQpBdjIgeHF05OCRPR2GkCmMB26PxlubOGPa+r7bd+IWmhido3tvcog01KXuuFB5GcHrbiyj/+mTj3bXqzcsaaAakHBJVPNGgFky1cFymsuM1JSxZTOoxgd4j0BMCHQn0HoH+E0GucnpQIRIw/7tS491vJqJnJ/dWX2swbUCaCEpqRDJvmUQG5ZIoyS3BGgpx12COM4fJWACJoQonbQPGHBIEBmRwKLntkBNhEMdnbnsN0giWlVwWJPExZjGXJnEGZBGzWZNa1G3LTqzBgClBA8lkiFyGer5k3YCJwBe4M/EqFmS6nt0grZbHm8pV5ZAdKAfTDR2bYkUUP0Bsi3e740jxpOv0x8QpHi/EVnx1gPZ4MfU9vjjXnWPhntZWjylnU8EIZ3uYj3tLeNHu+MTxp+36+4+qHlQRUE4d6sr6rk8lpLtSOXHbepmrCymsQ6oXoTJJrw+sI4HKY1tA8Y/eO2f/eEe9fCEDoKpnJs4BKeIcBhf9UPQ1nAYUnj/8PL9wpD/SMEJFavv3YR9mJEzbVcPcMO/UzdpKyUrQsrlSr5WsjXJt1bb7178vwDzzDxv8AIN6kfG8hJRRteullbIZIDD+D14ez53EbjbEPo23gE4Wuwt9Ipxc8vs2RGKGQcp/DcOjYXiqwv8LFn0DAAD//wMATgdfyVYIAAA=";

        Grid<ControllableVoxel> best = Utils.safelyDeserialize(serializedBest, Grid.class);

        List<Integer> avalanchesSpatialExtension = new ArrayList<>();
        List<Integer> avalanchesTemporalExtension = new ArrayList<>();

        double finalT = 30;
        double pulseDuration = 0.4;
        double avalancheThreshold = 0.0002;
        int binSize = 10;

        // task
        CriticalityEvaluator criticalityEvaluator = new CriticalityEvaluator(
                finalT, // task duration
                new Settings(), // default settings for the physics engine
                avalancheThreshold,
                binSize
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

            if (metrics.get(0) > 0) {
                avalanchesSpatialExtension.add(metrics.get(0).intValue());
            }
            if (metrics.get(1) > 0) {
                avalanchesTemporalExtension.add(metrics.get(1).intValue());
            }
        });

        int spatialSum = avalanchesSpatialExtension.stream()
                .mapToInt(Integer::intValue)
                .sum();
        int temporalSum = avalanchesTemporalExtension.stream()
                .mapToInt(Integer::intValue)
                .sum();

    }

}
