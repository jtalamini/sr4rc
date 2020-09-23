package it.units.erallab;

import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.core.controllers.Controller;
import it.units.erallab.hmsrobots.core.controllers.TimeFunctions;
import it.units.erallab.hmsrobots.core.objects.ControllableVoxel;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.Voxel;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Point2;
import it.units.erallab.hmsrobots.util.Utils;
import it.units.erallab.hmsrobots.viewers.GraphicsDrawer;
import it.units.erallab.hmsrobots.viewers.GridEpisodeRunner;
import it.units.erallab.hmsrobots.viewers.GridOnlineViewer;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.Problem;
import it.units.malelab.jgea.core.evolver.CMAESEvolver;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.StandardEvolver;
import it.units.malelab.jgea.core.evolver.stopcondition.Iterations;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.core.listener.MultiFileListenerFactory;
import it.units.malelab.jgea.core.listener.collector.*;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.selector.Tournament;
import it.units.malelab.jgea.core.selector.Worst;
import it.units.malelab.jgea.core.util.Misc;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.UniformCrossover;
import it.units.malelab.jgea.representation.sequence.bit.BitString;
import it.units.malelab.jgea.representation.sequence.numeric.GaussianMutation;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;
import org.apache.commons.lang3.SerializationUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.dyn4j.dynamics.Settings;
import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static it.units.malelab.jgea.core.util.Args.d;
import static it.units.malelab.jgea.core.util.Args.i;

public class SR4RC extends Worker {

    public SR4RC(String[] args) {
        super(args);
    }

    public static void main(String[] args) {
        new SR4RC(args);
    }

    private void validate(Grid<ControllableVoxel> body) {
        Controller<ControllableVoxel> testController = new TimeFunctions(Grid.create(
                body.getW(), body.getH(),
                (x, y) -> (Double t) -> Math.sin(-2 * Math.PI * t + Math.PI * ((double) x / (double) body.getW()))
        ));

        final Locomotion locomotion = new Locomotion(
                20,
                Locomotion.createTerrain("flat"),
                Lists.newArrayList(
                        Locomotion.Metric.TRAVEL_X_VELOCITY,
                        Locomotion.Metric.RELATIVE_CONTROL_POWER
                ),
                new Settings()
        );

        Robot<ControllableVoxel> robot = new Robot<>(testController, body);

        ScheduledExecutorService uiExecutor = Executors.newScheduledThreadPool(2);
        ExecutorService executor = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
        // configure the viewer
        GridOnlineViewer gridOnlineViewer = new GridOnlineViewer(
                Grid.create(1, 1, "Simulation"),
                uiExecutor,
                GraphicsDrawer.build().setConfigurable("drawers", List.of(
                        it.units.erallab.hmsrobots.viewers.drawers.Robot.build(),
                        it.units.erallab.hmsrobots.viewers.drawers.Voxel.build(),
                        it.units.erallab.hmsrobots.viewers.drawers.Ground.build()
                ))
        );
        // set the delay from the simulation to the viewer
        gridOnlineViewer.start(0);
        // run the simulations
        GridEpisodeRunner<Robot<?>> runner = new GridEpisodeRunner<>(
                Grid.create(1, 1, Pair.of("Robot", robot)),
                locomotion,
                gridOnlineViewer,
                executor
        );
        runner.run();
    }

    private double computeKSStatistics(List<Point2> empiricalDistribution, LinearRegression linearRegression) {
        double theoreticalCumSum = 0.0;
        double empiricalCumSum = 0.0;
        double maxDistance = 0.0;
        double currentDistance;
        for (int i = 0; i < empiricalDistribution.size(); i++) {
            Point2 point = empiricalDistribution.get(i);
            theoreticalCumSum += linearRegression.predict(point.x);
            empiricalCumSum += point.y;
            currentDistance = Math.abs(theoreticalCumSum - empiricalCumSum);
            if (currentDistance > maxDistance) {
                maxDistance = currentDistance;
            }
        }
        return maxDistance;
    }

    @Override
    public void run() {

        // SCHEDULE: sbatch --array=0-10 --nodes=1 -o logs/out.%A_%a.txt -e logs/err.%A_%a.txt go.breakable.sh
        // STATUS: squeue -u $USER
        // CANCEL: scancel

        // general parameters
        int randomSeed = i(a("randomSeed", "1"));
        int cacheSize = 10000;
        // problem-related parameters
        int gridSide = i(a("gridSize", "10"));
        double finalT = 30;
        double pulseDuration = 0.4;
        double avalancheThreshold = d(a("avalancheThreshold", "0.0002"));
        int binSize = 10;
        double gaussianThreshold = 0d;
        // evolutionary parameters
        int nGaussian = i(a("nGaussian", "10"));
        int popSize = 500;
        int iterations = 100;
        double mutationProb = 0.01;
        int tournamentSize = 10;

        MultiFileListenerFactory<Object, Grid<ControllableVoxel>, Double> statsListenerFactory = new MultiFileListenerFactory<>(
                a("dir", "."),
                a("statsFile", null)
        );

        // task
        ReservoirEvaluator reservoirEvaluator = new ReservoirEvaluator(
                finalT, // task duration
                ReservoirEvaluator.createTerrain("flat"),
                new Settings(), // default settings for the physics engine
                avalancheThreshold,
                binSize
        );

        Function<Robot<?>, List<Double>> task = Misc.cached(reservoirEvaluator, 10000);

        // voxel made of the soft material
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

        // problem
        Problem<Grid<ControllableVoxel>, Double> problem = () -> body -> {
            if (body.values().stream().noneMatch(Objects::nonNull)) {
                return 0.0;
            }
            List<Integer> avalanchesSpatialExtension = new ArrayList<>();
            List<Integer> avalanchesTemporalExtension = new ArrayList<>();
            // a pulse controller is applied on each voxel
            IntStream.range(0, (int) Math.pow(gridSide, 2d)).forEach(i -> {
                Controller<ControllableVoxel> pulseController = new TimeFunctions(Grid.create(gridSide, gridSide, (x, y) -> (Double t) -> {
                    if (x * gridSide + y == i) {
                        if (t < pulseDuration/2) {
                            return 1.0;
                        } else if (t < pulseDuration) {
                            return -1.0;
                        }
                    }
                    return 0.0;
                }));
                List<Double> metrics = task.apply(new Robot<>(pulseController, SerializationUtils.clone(body)));

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

            List<Point2> logLogSpatialDistribution = avalanchesSpatialExtension.stream()
                    .distinct()
                    .map(avalanche -> Point2.build(Math.log(avalanche), Math.log(Collections.frequency(avalanchesSpatialExtension, avalanche))/spatialSum))
                    .collect(Collectors.toList());

            List<Point2> logLogTemporalDistribution = avalanchesTemporalExtension.stream()
                    .distinct()
                    .map(avalanche -> Point2.build(Math.log(avalanche), Math.log(Collections.frequency(avalanchesTemporalExtension, avalanche))/temporalSum))
                    .collect(Collectors.toList());

            if ((logLogSpatialDistribution.size() < 2) || (logLogTemporalDistribution.size() < 2)) {
                return 0.0;
            }

            // linear regression of the log-log distribution (only the first 10 non-empty bins are taken)
            LinearRegression spatialLinearRegression = new LinearRegression(logLogSpatialDistribution);
            LinearRegression temporalLinearRegression = new LinearRegression(logLogTemporalDistribution);
            double RSquared = (spatialLinearRegression.R2() + temporalLinearRegression.R2())/2;

            // 3. KS statistics
            double ks1 = computeKSStatistics(logLogSpatialDistribution, spatialLinearRegression);
            double ks2 = computeKSStatistics(logLogTemporalDistribution, temporalLinearRegression);
            double DSquared = Math.pow(Math.exp(-(0.9 * Math.min(ks1, ks2) + 0.1 * (ks1 + ks2)/2)), 2d);

            return RSquared + DSquared;
        };

        // direct mapper
        Function<BitString, Grid<ControllableVoxel>> directMapper = g -> {
            if (g.asBitSet().stream().sum() == 0) {
                return null;
            }
            return Utils.gridLargestConnected(Grid.create(gridSide, gridSide, (x, y) -> g.get(gridSide * x + y) ? SerializationUtils.clone(softMaterial) : null), Objects::nonNull);
        };

        // gaussian mapper
        Function<List<Double>, Grid<ControllableVoxel>> gaussianMapper = g -> {
            Grid<Double> gaussianGrid = Grid.create(gridSide, gridSide, 0d);
            double epsilon = 0.1;
            int c = 0;
            for (int j = 0; j < nGaussian; j++) {
                //extract parameter of the j-th gaussian for the i-th material
                double muX = g.get(c + 0);
                double muY = g.get(c + 1);
                double sigmaX = Math.max(0d, g.get(c + 2)) + epsilon;
                double sigmaY =  Math.max(0d, g.get(c + 3)) + epsilon;
                double weight = g.get(c + 4) * 2d -1;
                c = c + 5;
                //compute over grid
                for (int ix = 0; ix < gridSide; ix++) {
                    for (int iy = 0; iy < gridSide; iy++) {
                        double x = (double)ix/(double)gridSide;
                        double y = (double)iy/(double)gridSide;
                        gaussianGrid.set(ix, iy, gaussianGrid.get(ix, iy) + weight * Math.exp(-(Math.pow(x - muX, 2d)/(2.0 * Math.pow(sigmaX, 2d)) + Math.pow(y - muY, 2d)/(2.0 * Math.pow(sigmaY, 2d)))));
                    }
                }
            }
            //build grid with material index
            Grid<ControllableVoxel> body = Grid.create(gridSide, gridSide, (x, y) -> gaussianGrid.get(x, y) > gaussianThreshold ? SerializationUtils.clone(softMaterial) : null);
            //find largest connected and crop
            body = Utils.gridLargestConnected(body, i -> i != null);
            body = Utils.cropGrid(body, i -> i != null);
            return body;
        };

        // old evolver
        Evolver<List<Double>, Grid<ControllableVoxel>, Double> evolver = new StandardEvolver<>(
                gaussianMapper,
                new FixedLengthListFactory<>(nGaussian * 5, new UniformDoubleFactory(0, 1)),
                PartialComparator.from(Double.class).reversed().comparing(Individual::getFitness), // fitness comparator
                popSize, // pop size
                Map.of(
                        new GaussianMutation(mutationProb), 0.2d,
                        new UniformCrossover<>(new FixedLengthListFactory<>(nGaussian * 5, new UniformDoubleFactory(0, 1))), 0.8d
                ),
                new Tournament(tournamentSize), // depends on pop size
                new Worst(), // worst individual dies
                popSize,
                true
        );

        // CMA-ES evolver: https://en.wikipedia.org/wiki/CMA-ES
        Evolver<List<Double>, Grid<ControllableVoxel>, Double> cmaesEvolver = new CMAESEvolver<>(
                gaussianMapper,
                new FixedLengthListFactory<>(nGaussian * 5, new UniformDoubleFactory(0, 1)),
                PartialComparator.from(Double.class).reversed().comparing(Individual::getFitness),
                0,
                1
        );

        List<DataCollector<?, ? super Grid<ControllableVoxel>, ? super Double>> collectors = List.of(
                new Basic(),
                new Population(),
                new Diversity(),
                new BestInfo("%6.4f"),
                new FunctionOfOneBest<>(i -> List.of(
                        new Item("serialized.grid", it.units.erallab.Utils.safelySerialize(i.getSolution()), "%s")
                ))
        );

        Listener<? super Object, ? super Grid<ControllableVoxel>, ? super Double> listener;
        if (statsListenerFactory.getBaseFileName() == null) {
            listener = listener(collectors.toArray(DataCollector[]::new));
        } else {
            listener = statsListenerFactory.build(collectors.toArray(DataCollector[]::new));
        }

        // optimization
        try {
            Collection<Grid<ControllableVoxel>> solutions = cmaesEvolver.solve(
                    Misc.cached(problem.getFitnessFunction(), cacheSize),
                    new Iterations(iterations),
                    new Random(randomSeed),
                    executorService,
                    listener
            );
            // print one solution
            if (solutions.size() > 0) {
                Predicate<ControllableVoxel> predicate = voxel -> voxel != null;
                Grid<ControllableVoxel> best = (Grid<ControllableVoxel>) solutions.toArray()[0];
                System.out.println(Grid.toString(best, predicate));
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        } catch (ExecutionException e) {
            e.printStackTrace();
        }
    }
}