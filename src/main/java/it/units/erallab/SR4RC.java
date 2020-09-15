package it.units.erallab;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.core.controllers.Controller;
import it.units.erallab.hmsrobots.core.controllers.TimeFunctions;
import it.units.erallab.hmsrobots.core.objects.ControllableVoxel;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.Voxel;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Point2;
import it.units.erallab.hmsrobots.util.Utils;
import it.units.erallab.hmsrobots.viewers.GraphicsDrawer;
import it.units.erallab.hmsrobots.viewers.GridEpisodeRunner;
import it.units.erallab.hmsrobots.viewers.GridOnlineViewer;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.Problem;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.StandardEvolver;
import it.units.malelab.jgea.core.evolver.stopcondition.Iterations;
import it.units.malelab.jgea.core.listener.collector.Basic;
import it.units.malelab.jgea.core.listener.collector.BestInfo;
import it.units.malelab.jgea.core.listener.collector.Diversity;
import it.units.malelab.jgea.core.listener.collector.Population;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.selector.Tournament;
import it.units.malelab.jgea.core.selector.Worst;
import it.units.malelab.jgea.core.util.Misc;
import it.units.malelab.jgea.representation.sequence.UniformCrossover;
import it.units.malelab.jgea.representation.sequence.bit.BitFlipMutation;
import it.units.malelab.jgea.representation.sequence.bit.BitString;
import it.units.malelab.jgea.representation.sequence.bit.BitStringFactory;
import org.apache.commons.lang3.SerializationUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.math3.stat.inference.KolmogorovSmirnovTest;
import org.dyn4j.dynamics.Settings;
import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class SR4RC extends Worker {

    public SR4RC(String[] args) {
        super(args);
    }

    public static void main(String[] args) {

        Controller<ControllableVoxel> pulseController = new TimeFunctions(Grid.create(5, 5, (x, y) -> (Double t) -> {
            if ((t < 0.05) && (x * 5 + y == 0)) {
                return 1.0;
            } else {
                return 0.0;
            }
        }));

        ReservoirEvaluator reservoirEvaluator = new ReservoirEvaluator(
                20, // task duration
                ReservoirEvaluator.createTerrain("flat"),
                new Settings() // default settings for the physics engine
        );

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

        Grid<ControllableVoxel> body = Grid.create(5,5, (x,y) -> SerializationUtils.clone(softMaterial));

        Robot<ControllableVoxel> robot = new Robot<>(pulseController, body);

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
                reservoirEvaluator,
                gridOnlineViewer,
                executor
        );
        runner.run();

        //new SR4RC(args);



    }

    @Override
    public void run() {
        // parameters
        int width = 5;
        int height = 5;
        int populationSize = 500;
        double mutationProb = 0.01;
        int tournamentSize = 10;
        int cacheSize = 10000;
        int iterations = 100;
        int randomSeed = 0;
        double finalT = 10;

        // task
        ReservoirEvaluator reservoirEvaluator = new ReservoirEvaluator(
                finalT, // task duration
                ReservoirEvaluator.createTerrain("flat"),
                new Settings() // default settings for the physics engine
        );

        // problem
        Problem<Grid<ControllableVoxel>, Double> problem = () -> body -> {
            List<Double> avalanches = new ArrayList<>();
            // a pulse controller is applied on each voxel
            IntStream.range(0, width * height).forEach(i -> {
                Controller<ControllableVoxel> pulseController = new TimeFunctions(Grid.create(width, height, (x, y) -> (Double t) -> {
                    if ((t < 0.05) && (x * height + y == i)) {
                        return 1.0;
                    } else {
                        return 0.0;
                    }
                }));
                List<Double> metrics = reservoirEvaluator.apply(new Robot<>(pulseController, SerializationUtils.clone(body)));
                avalanches.add(metrics.get(0));
            });
            // here create the avalanches distribution and measure distance from self-organized criticality
            Map<Double, Long> avalanchesDistribution = avalanches.stream()
                    .collect(Collectors.groupingBy(avalanche -> avalanche, Collectors.counting()));

            // the distribution should fit a powerlaw distribution
            // 1. linear regression of the log-log distribution
            List<Point2> logLogData = avalanchesDistribution.entrySet().stream()
                    .map(entry -> Point2.build(Math.log(entry.getValue()), Math.log(entry.getKey())))
                    .collect(Collectors.toList());
            LinearRegression lr = new LinearRegression(logLogData);
            double determinationCoefficient = lr.R2();
            // 2. bins ????
            /*
            double max = avalanchesDistribution.entrySet().stream()
                    .mapToDouble(Double::doubleValue)
                    .max();
            double average = avalanchesDistribution.entrySet().stream()
                    .mapToDouble(Double::doubleValue)
                    .average();
            double binsCoefficient = Math.tanh(5 * (0.9 * max) + 0.1 * average);
             */
            // 3. KS statistics
            KolmogorovSmirnovTest ks = new KolmogorovSmirnovTest();
            double[] fitArray = new double[logLogData.size()];
            double[] realArray = new double[logLogData.size()];
            int index = 0;
            for (Point2 point : logLogData) {
                fitArray[index] = lr.predict(point.x);
                realArray[index] = point.x;
                index ++;
            }
            double kolmogorovSmirnovStatistic = ks.kolmogorovSmirnovStatistic(fitArray, realArray);
            double ksStatisticsCoefficient = Math.exp(-(0.9 * kolmogorovSmirnovStatistic + 0.1 * kolmogorovSmirnovStatistic));
            // 4. unique states

            // 5. log likelihood

            return determinationCoefficient + kolmogorovSmirnovStatistic;

        };

        // mapper
        //Function<BitString, Grid<ControllableVoxel>> mapper = g -> Utils.gridLargestConnected(Grid.create(width, height, (x, y) -> g.get(height * x + y) ? new ControllableVoxel() : null), Objects::nonNull);
        Function<BitString, Grid<ControllableVoxel>> mapper = g -> {
            if (g.asBitSet().stream().sum() == 0) {
                g.set(0, true);
            }
            return Utils.gridLargestConnected(Grid.create(width, height, (x, y) -> g.get(height * x + y) ? new ControllableVoxel() : null), Objects::nonNull);
        };

        // evolver
        Evolver<BitString, Grid<ControllableVoxel>, Double> evolver = new StandardEvolver<>(
                mapper,
                new BitStringFactory(width * height), // w x h
                PartialComparator.from(Double.class).comparing(Individual::getFitness), // fitness comparator
                populationSize, // pop size
                Map.of(
                        new BitFlipMutation(mutationProb), 0.2d,
                        new UniformCrossover<>(new BitStringFactory(width * height)), 0.8d
                ),
                new Tournament(tournamentSize), // depends on pop size
                new Worst(), // worst individual dies
                populationSize,
                true
        );

        // optimization
        try {
            evolver.solve(
                    Misc.cached(problem.getFitnessFunction(), cacheSize),
                    new Iterations(iterations),
                    new Random(randomSeed),
                    executorService,
                    listener(
                            new Basic(),
                            new Population(),
                            new Diversity(),
                            new BestInfo("%5.1f")
                    )
            );
        } catch (InterruptedException e) {
            e.printStackTrace();
        } catch (ExecutionException e) {
            e.printStackTrace();
        }
    }
}