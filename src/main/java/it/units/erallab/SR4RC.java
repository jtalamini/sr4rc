package it.units.erallab;

import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.core.controllers.Controller;
import it.units.erallab.hmsrobots.core.controllers.TimeFunctions;
import it.units.erallab.hmsrobots.core.objects.ControllableVoxel;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Utils;
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
import org.dyn4j.dynamics.Settings;
import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class SR4RC extends Worker {

    public SR4RC(String[] args) {
        super(args);
    }

    public static void main(String[] args) {
        new SR4RC(args);
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
                Lists.newArrayList(ReservoirEvaluator.Metric.SELF_ORGANIZED_CRITICALITY),
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
            // here create the avalanches distribution and measure distance from self-organized criticality slope
            Map<Double, Long> avalanchesDistribution = avalanches.stream()
                    .collect(Collectors.groupingBy(avalanche -> avalanche, Collectors.counting()));

            // the distribution should fit a symmetric line with slope -1
            // optimally: y + x = 0

            double distanceFromCriticality = avalanchesDistribution.entrySet().stream()
                    .map(entry -> Math.pow(Math.log(entry.getValue()) + Math.log(entry.getKey()), 2))
                    .mapToDouble(Double::doubleValue)
                    .sum();
            return distanceFromCriticality;
        };

        // mapper
        Function<BitString, Grid<ControllableVoxel>> mapper = g -> Utils.gridLargestConnected(Grid.create(width, height, (x, y) -> g.get(height * x + y) ? new ControllableVoxel() : null), Objects::nonNull);

        // evolver
        Evolver<BitString, Grid<ControllableVoxel>, Double> evolver = new StandardEvolver<>(
                mapper,
                new BitStringFactory(width * height), // w x h
                PartialComparator.from(Double.class).reversed().comparing(Individual::getFitness), // fitness comparator
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