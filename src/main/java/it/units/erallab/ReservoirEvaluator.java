package it.units.erallab;

import it.units.erallab.hmsrobots.core.objects.Ground;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.WorldObject;
import it.units.erallab.hmsrobots.core.objects.immutable.Snapshot;
import it.units.erallab.hmsrobots.core.objects.immutable.Voxel;
import it.units.erallab.hmsrobots.tasks.AbstractTask;
import it.units.erallab.hmsrobots.util.BoundingBox;
import it.units.erallab.hmsrobots.util.Point2;
import it.units.erallab.hmsrobots.viewers.SnapshotListener;
import org.dyn4j.dynamics.Settings;
import org.dyn4j.dynamics.World;
import org.dyn4j.geometry.Vector2;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class ReservoirEvaluator extends AbstractTask<Robot<?>, List<Double>> {
    private final double finalT;
    private final double[][] groundProfile;
    private final double initialPlacement;
    private final List<ReservoirEvaluator.Metric> metrics;
    private double threshold;

    public ReservoirEvaluator(double finalT, double[][] groundProfile, List<Metric> metrics, Settings settings) {
        super(settings);
        // gravità o misurare transitorio?
        this.finalT = finalT;
        this.groundProfile = groundProfile;
        this.initialPlacement = groundProfile[0][1] + 1.0D;
        this.metrics = metrics;
        this.threshold = 0.0001d;
    }

    public List<Double> apply(Robot<?> robot, SnapshotListener listener) {
        // this is the map containing the states visited during the simulation
        Map<List<Integer>, Double> edgeOfChaosByAreaMap = new HashMap<>();
        Map<Point2, Double> edgeOfChaosByCmMap = new HashMap<>();

        List<Double> previousAreas = new ArrayList<>();
        List<Integer> avalanche = new ArrayList<>();
        int acviteVoxels;

        World world = new World();
        world.setSettings(this.settings);
        List<WorldObject> worldObjects = new ArrayList();
        Ground ground = new Ground(this.groundProfile[0], this.groundProfile[1]);
        ground.addTo(world);
        worldObjects.add(ground);

        BoundingBox boundingBox = robot.boundingBox();

        robot.translate(new Vector2(this.initialPlacement - boundingBox.min.x, 0.0D));

        double minYGap = robot.getVoxels().values().stream()
                .filter(Objects::nonNull)
                .mapToDouble((v) -> ((Voxel) v.immutable()).getShape().boundingBox().min.y - ground.yAt(v.getCenter().x))
                .min()
                .orElse(0.0D);

        robot.translate(new Vector2(0.0D, 1.0D - minYGap));

        robot.addTo(world);
        worldObjects.add(robot);

        double t = 0.0D;
        while (t < this.finalT) {
            t += this.settings.getStepFrequency();
            world.step(1);
            robot.act(t);

            // edge-of-chaos
            if (this.metrics.contains(Metric.EDGE_OF_CHAOS_BY_AREA)) {
                List<Integer> voxelsArea = robot.getVoxels().values().stream()
                        .map(voxel -> voxel != null ? (int)Math.round((voxel).getAreaRatio()*10) : 0)
                        .collect(Collectors.toList());
                voxelsArea.stream().forEach(area -> System.out.print(area+" "));
                System.out.println();
                if (edgeOfChaosByAreaMap.keySet().stream().anyMatch(k -> k.equals(voxelsArea))) {
                    break;
                } else {
                    edgeOfChaosByAreaMap.put(voxelsArea, t);
                }
            }
            if (this.metrics.contains(Metric.EDGE_OF_CHAOS_BY_CM)) {
                Point2 cm = Point2.build( (int) robot.getCenter().x, (int) robot.getCenter().y);
                if (edgeOfChaosByCmMap.keySet().stream()
                        .anyMatch(k -> ((k.x == cm.x) && (k.y == cm.y)))
                ) {
                    break;
                } else {
                    edgeOfChaosByCmMap.put(cm, t);
                }
            }

            // self-organized criticality
            // 1. count how many voxels have changed shape since last step
            // 2. count how long this process occurs
            if (this.metrics.contains(Metric.SELF_ORGANIZED_CRITICALITY)) {
                List<Double> currentAreas = robot.getVoxels().values().stream()
                        .filter(Objects::nonNull)
                        .map(it.units.erallab.hmsrobots.core.objects.Voxel::getAreaRatio)
                        .collect(Collectors.toList());

                List<Double> finalPreviousAreas = previousAreas;
                if (previousAreas.size() > 0) {
                    acviteVoxels = (int) IntStream.range(0, previousAreas.size())
                            .filter(i -> Math.abs(finalPreviousAreas.get(i) - currentAreas.get(i)) > threshold)
                            .count();
                } else {
                    acviteVoxels = currentAreas.size();
                }

                if (acviteVoxels == 0) {
                    break;
                } else {
                    avalanche.add(acviteVoxels);
                    previousAreas = currentAreas;
                }
            }

            // this saves the robot info during the simulation
            if (listener != null) {
                Snapshot snapshot = new Snapshot(t, (Collection) worldObjects.stream().map(WorldObject::immutable).collect(Collectors.toList()));
                listener.listen(snapshot);
            }
        }

        List<Double> results = new ArrayList(this.metrics.size());
        double value;
        for (Iterator iterator = this.metrics.iterator(); iterator.hasNext(); results.add(value)) {
            ReservoirEvaluator.Metric metric = (ReservoirEvaluator.Metric) iterator.next();
            value = 0.0D / 0.0;
            switch (metric) {
                case EDGE_OF_CHAOS_BY_AREA:
                    value = edgeOfChaosByAreaMap.entrySet().size();
                    break;
                case EDGE_OF_CHAOS_BY_CM:
                    value = edgeOfChaosByCmMap.entrySet().size();
                    break;
                case SELF_ORGANIZED_CRITICALITY:
                    value = avalanche.stream().mapToInt(Integer::intValue).sum();
                    break;
            }
        }
        return results;
    }

    private static double[][] randomTerrain(int n, double length, double peak, double borderHeight, Random random) {
        double[] xs = new double[n + 2];
        double[] ys = new double[n + 2];
        xs[0] = 0.0D;
        xs[n + 1] = length;
        ys[0] = borderHeight;
        ys[n + 1] = borderHeight;

        for(int i = 1; i < n + 1; ++i) {
            xs[i] = 1.0D + (double)(i - 1) * (length - 2.0D) / (double)n;
            ys[i] = random.nextDouble() * peak;
        }

        return new double[][]{xs, ys};
    }

    /*
    this will be removed if the INPUT is the ground slope
     */
    public static double[][] createTerrain(String name) {
        Random random = new Random(1L);
        if (name.equals("flat")) {
            return new double[][]{{0.0D, 10.0D, 1990.0D, 2000.0D}, {100.0D, 0.0D, 0.0D, 100.0D}};
        } else if (name.startsWith("uneven")) {
            int h = Integer.parseInt(name.replace("uneven", ""));
            return randomTerrain(50, 2000.0D, (double)h, 100.0D, random);
        } else {
            return null;
        }
    }

    public List<ReservoirEvaluator.Metric> getMetrics() {
        return this.metrics;
    }

    /*
    here we put the reservoir output state that can be measured
     */
    public static enum Metric {
        EDGE_OF_CHAOS_BY_AREA(false),
        EDGE_OF_CHAOS_BY_CM(false),
        SELF_ORGANIZED_CRITICALITY(false);

        private final boolean toMinimize;

        private Metric(boolean toMinimize) {
            this.toMinimize = toMinimize;
        }

        public boolean isToMinimize() {
            return this.toMinimize;
        }
    }
}
