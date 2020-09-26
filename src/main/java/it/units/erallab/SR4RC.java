package it.units.erallab;

import it.units.erallab.hmsrobots.core.controllers.Controller;
import it.units.erallab.hmsrobots.core.controllers.TimeFunctions;
import it.units.erallab.hmsrobots.core.objects.ControllableVoxel;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.Voxel;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Point2;
import it.units.erallab.hmsrobots.util.Utils;
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
import it.units.malelab.jgea.representation.sequence.bit.BitFlipMutation;
import it.units.malelab.jgea.representation.sequence.bit.BitString;
import it.units.malelab.jgea.representation.sequence.bit.BitStringFactory;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;
import org.apache.commons.lang3.SerializationUtils;
import org.dyn4j.dynamics.Settings;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutionException;
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

    private String printDistribution(List<Point2> distribution) {
        return distribution.stream()
                .map(point -> point.x+":"+point.y)
                .collect(Collectors.joining(" "));
    }

    private String printBody(Grid<ControllableVoxel> bestBody) {
        String bestBodyString = "";
        for (int y = 0; y < bestBody.getH(); y++) {
            for (int x = 0; x < bestBody.getW(); x++) {
                bestBodyString += bestBody.get(x, y) == null ? "0" : "X";
                bestBodyString += x == bestBody.getW()-1 ? "n " : " ";
            }
        }
        return bestBodyString;
    }

    private String testBest(Grid<ControllableVoxel> best, double pulseDuration, CriticalityEvaluator task, int binSize) {
        int[] avalanchesSpatialExtension = new int[best.getW() * best.getW()];
        int[] avalanchesTemporalExtension = new int[1000];

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
            List<Double> metrics = task.apply(new Robot<>(pulseController, SerializationUtils.clone(best)));

            avalanchesSpatialExtension[metrics.get(0).intValue()] += 1;
            avalanchesTemporalExtension[(metrics.get(1).intValue()) / binSize] += 1;
        });

        // create 2 normalized distributions for each individual
        double[] spatialDistribution =  Arrays.stream(avalanchesSpatialExtension)
                .mapToDouble(frequency -> frequency / (double)(best.getW() * best.getW()))
                .toArray();

        double[] temporalDistribution = Arrays.stream(avalanchesTemporalExtension)
                .mapToDouble(frequency -> frequency / (double)(best.getW() * best.getW()))
                .toArray();

        String distributions = "space ";
        // compute the log-log of the 2 distributions
        List<Point2> logLogSpatialDistribution = IntStream.range(1, spatialDistribution.length)
                .mapToObj(i -> Point2.build(Math.log10(i), spatialDistribution[i] > 0.0 ? Math.log10(spatialDistribution[i]) : 0))
                .collect(Collectors.toList());
        distributions += printDistribution(logLogSpatialDistribution);

        List<Point2> logLogTemporalDistribution = IntStream.range(1, temporalDistribution.length)
                .mapToObj(i -> Point2.build(Math.log10((i+1) * binSize), temporalDistribution[i] > 0.0 ? Math.log10(temporalDistribution[i]) : 0))
                .collect(Collectors.toList());
        distributions += " time "+printDistribution(logLogTemporalDistribution);
        return distributions;
    }


    @Override
    public void run() {

        Map<Grid<ControllableVoxel>, List<Point2>> spatialDistributions = new ConcurrentHashMap<>();
        Map<Grid<ControllableVoxel>, List<Point2>> temporalDistributions = new ConcurrentHashMap<>();

        // SCHEDULE: sbatch --array=0-10 --nodes=1 -o logs/out.%A_%a.txt -e logs/err.%A_%a.txt sr4rc.sh
        // STATUS: squeue -u $USER
        // CANCEL: scancel

        // general parameters
        int randomSeed = i(a("randomSeed", "666"));
        int cacheSize = 10000;
        // problem-related parameters
        int gridSide = i(a("gridSize", "5"));
        double finalT = 30;
        double pulseDuration = 0.4;
        double avalancheThreshold = d(a("avalancheThreshold", "0.0002"));
        int binSize = 10;
        double gaussianThreshold = 0d;
        // evolutionary parameters
        int nGaussian = i(a("nGaussian", "10"));
        String evolverType = a("evolver", "direct");
        int popSize = 500;
        int iterations = 100;
        double mutationProb = 0.01;
        int tournamentSize = 10;

        MultiFileListenerFactory<Object, Grid<ControllableVoxel>, Double> statsListenerFactory = new MultiFileListenerFactory<>(
                a("dir", "."),
                a("statsFile", null)
        );

        // task
        CriticalityEvaluator criticalityEvaluator = new CriticalityEvaluator(
                finalT, // task duration
                new Settings(), // default settings for the physics engine
                avalancheThreshold,
                binSize
        );

        Function<Robot<?>, List<Double>> task = Misc.cached(criticalityEvaluator, 10000);

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

            int[] avalanchesSpatialExtension = new int[gridSide * gridSide];
            int[] avalanchesTemporalExtension = new int[1000];

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

                avalanchesSpatialExtension[metrics.get(0).intValue()] += 1;
                avalanchesTemporalExtension[(metrics.get(1).intValue()) / binSize] += 1;
            });

            // exit condition
            int spatialSizeNumber = (int) Arrays.stream(avalanchesSpatialExtension)
                    .filter(frequency -> frequency > 0)
                    .count();
            int temporalSizeNumber = (int) Arrays.stream(avalanchesTemporalExtension)
                    .filter(frequency -> frequency > 0)
                    .count();
            if (spatialSizeNumber < 2 || temporalSizeNumber < 2) {
                return 0.0;
            }

            // create 2 normalized distributions for each individual
            double[] spatialDistribution =  Arrays.stream(avalanchesSpatialExtension)
                    .mapToDouble(frequency -> frequency / (double)(gridSide * gridSide))
                    .toArray();

            double[] temporalDistribution = Arrays.stream(avalanchesTemporalExtension)
                    .mapToDouble(frequency -> frequency / (double)(gridSide * gridSide))
                    .toArray();

            // compute the log-log of the 2 distributions
            List<Point2> logLogSpatialDistribution = IntStream.range(1, spatialDistribution.length)
                    .mapToObj(i -> Point2.build(Math.log10(i), spatialDistribution[i] > 0.0 ? Math.log10(spatialDistribution[i]) : 0))
                    .collect(Collectors.toList());
            spatialDistributions.put(body, logLogSpatialDistribution);
            List<Point2> logLogTemporalDistribution = IntStream.range(1, temporalDistribution.length)
                    .mapToObj(i -> Point2.build(Math.log10((i+1) * binSize), temporalDistribution[i] > 0.0 ? Math.log10(temporalDistribution[i]) : 0))
                    .collect(Collectors.toList());
            temporalDistributions.put(body, logLogTemporalDistribution);

            // linear regression of the log-log distribution
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
            double epsilon = 0.0001;
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
            body = Utils.gridLargestConnected(body, Objects::nonNull);
            body = Utils.cropGrid(body, Objects::nonNull);
            return body;
        };

        // old evolver
        Evolver<BitString, Grid<ControllableVoxel>, Double> directEvolver = new StandardEvolver<>(
                directMapper,
                new BitStringFactory(gridSide * gridSide),
                PartialComparator.from(Double.class).reversed().comparing(Individual::getFitness), // fitness comparator
                popSize, // pop size
                Map.of(
                        new BitFlipMutation(mutationProb), 0.2d,
                        new UniformCrossover<>(new BitStringFactory(gridSide * gridSide)), 0.8d
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
                        new Item("serialized.grid", it.units.erallab.Utils.safelySerialize(i.getSolution()), "%s"),
                        new Item("distributions", testBest(i.getSolution(), pulseDuration, criticalityEvaluator, binSize), "%s"),
                        new Item("body", printBody(i.getSolution()), "%s")
                ))
        );

        Listener<? super Object, ? super Grid<ControllableVoxel>, ? super Double> listener;
        if (statsListenerFactory.getBaseFileName() == null) {
            listener = listener(collectors.toArray(DataCollector[]::new));
        } else {
            listener = statsListenerFactory.build(collectors.toArray(DataCollector[]::new));
        }

        // optimization
        Collection<Grid<ControllableVoxel>> solutions = new ArrayList<>();
        try {
            if (evolverType.equals("direct")) {
                solutions = directEvolver.solve(
                        Misc.cached(problem.getFitnessFunction(), cacheSize),
                        new Iterations(iterations),
                        new Random(randomSeed),
                        executorService,
                        listener
                );
            } else if (evolverType.equals("cmaes")) {
                solutions = cmaesEvolver.solve(
                        Misc.cached(problem.getFitnessFunction(), cacheSize),
                        new Iterations(iterations),
                        new Random(randomSeed),
                        executorService,
                        listener
                );
            }
            // print one solution
            if (solutions.size() > 0) {
                Predicate<ControllableVoxel> predicate = Objects::nonNull;
                Grid<ControllableVoxel> best = (Grid<ControllableVoxel>) solutions.toArray()[0];
                System.out.println(Grid.toString(best, predicate));
            }
        } catch (InterruptedException | ExecutionException e) {
            e.printStackTrace();
        }
    }
}