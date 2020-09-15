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
    private double threshold;

    public ReservoirEvaluator(double finalT, double[][] groundProfile, Settings settings) {
        super(settings);
        // gravità o misurare transitorio?
        this.finalT = finalT;
        this.groundProfile = groundProfile;
        this.initialPlacement = groundProfile[0][1] + 1.0D;
        this.threshold = 0.00001d;
    }

    public List<Double> apply(Robot<?> robot, SnapshotListener listener) {

        List<Double> previousAreas = null;
        int[] avalanchedVoxels = new int[robot.getVoxels().getW()*robot.getVoxels().getH()];
        int avalanchesTemporalExtension = 0;
        int avalanchesSpatialExtension;

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
            // self-organized criticality
            // 1. count how many voxels have changed shape since last step
            // 2. count how long this process occurs
            List<Double> currentAreas = robot.getVoxels().values().stream()
                    .filter(Objects::nonNull)
                    .map(it.units.erallab.hmsrobots.core.objects.Voxel::getAreaRatio)
                    .collect(Collectors.toList());

            List<Double> finalPreviousAreas = previousAreas;
            if (previousAreas != null) {
                int[] activeVoxelsIndex = IntStream.range(0, previousAreas.size())
                        .filter(i -> Math.abs(finalPreviousAreas.get(i) - currentAreas.get(i)) > threshold)
                        .toArray();

                if (activeVoxelsIndex.length == 0) {
                    break;
                } else {
                    avalanchesTemporalExtension += 1;
                    //  here compute the avalanche spatial extension
                    Arrays.stream(activeVoxelsIndex).forEach(i -> avalanchedVoxels[i] = 1);
                }
            }
            previousAreas = currentAreas;

            // this saves the robot info during the simulation
            if (listener != null) {
                Snapshot snapshot = new Snapshot(t, (Collection) worldObjects.stream().map(WorldObject::immutable).collect(Collectors.toList()));
                listener.listen(snapshot);
            }
        }
        avalanchesSpatialExtension = Arrays.stream(avalanchedVoxels).sum();

        List results = new ArrayList();
        results.add(avalanchesSpatialExtension);
        results.add(avalanchesTemporalExtension);
        System.out.println("Spatial extension: "+avalanchesSpatialExtension);
        System.out.println("Temporal extension: "+avalanchesTemporalExtension);
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
}
