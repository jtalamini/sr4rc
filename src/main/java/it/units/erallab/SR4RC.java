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
import it.units.malelab.jgea.core.evolver.stopcondition.Births;
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
import it.units.malelab.jgea.representation.sequence.numeric.GaussianMutation;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;
import org.apache.commons.lang3.SerializationUtils;
import org.dyn4j.dynamics.Settings;
import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static it.units.malelab.jgea.core.util.Args.i;

public class SR4RC extends Worker {

    public SR4RC(String[] args) {
        super(args);
    }

    public static void main(String[] args) {
        new SR4RC(args);
    }

    public static double computeKSStatistics(List<Point2> empiricalDistribution, LinearRegression linearRegression) {
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

    private String printDistribution(double[] distribution) {
        return Arrays.stream(distribution)
                .mapToObj(value -> ""+value)
                .collect(Collectors.joining(" "));
    }

    private String bodyToString(Grid<ControllableVoxel> bestBody) {
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

        int bodySize = (int) best.values().stream().filter(Objects::nonNull).count();

        double[] avalanchesSpatialExtension = new double[bodySize + 1];
        double[] avalanchesTemporalExtension = new double[1000];
        // a pulse controller is applied on each voxel
        best.stream()
                .filter(Objects::nonNull)
                .forEach(voxel -> {
                    Controller<ControllableVoxel> pulseController = new TimeFunctions(Grid.create(best.getW(), best.getH(), (x, y) -> (Double t) -> {
                        if (x == voxel.getX() && y == voxel.getY()) {
                            if (t < pulseDuration/2) {
                                return 1.0;
                            } else if (t < pulseDuration) {
                                return -1.0;
                            }
                        }
                        return 0.0;
                    }));
                    List<Double> metrics = task.apply(new Robot<>(pulseController, SerializationUtils.clone(best)));

                    if (metrics.get(0).intValue() > 0) {
                        avalanchesSpatialExtension[metrics.get(0).intValue()] += 1;
                    }
                    if (metrics.get(1).intValue() > 0) {
                        avalanchesTemporalExtension[(metrics.get(1).intValue()) / binSize] += 1;
                    }
                });
        String distributions = "";
        distributions += printDistribution(avalanchesSpatialExtension);
        distributions += ":"+printDistribution(avalanchesTemporalExtension);
        return distributions;
    }


    @Override
    public void run() {

        // SCHEDULE: sbatch --array=0-10 --nodes=1 -o logs/out.%A_%a.txt -e logs/err.%A_%a.txt sr4rc.sh
        // STATUS: squeue -u $USER
        // CANCEL: scancel

        // general parameters
        int randomSeed = i(a("randomSeed", "77"));
        int cacheSize = 10000;
        // problem-related parameters
        int gridSide = i(a("gridSize", "5"));
        double finalT = 30;
        double pulseDuration = 0.4;
        int binSize = i(a("binSize", "5"));
        int robotVoxels = i(a("robotVoxels", "20"));
        // evolutionary parameters
        int nGaussian = i(a("nGaussian", "10"));
        String evolverType = a("evolver", "direct");
        int popSize = i(a("popSize", "500"));
        int iterations = i(a("iterations", "100"));
        int birth = i(a("births", "5000"));
        double mutationProb = 0.01;
        int tournamentSize = 10;

        MultiFileListenerFactory<Object, Grid<ControllableVoxel>, Double> statsListenerFactory = new MultiFileListenerFactory<>(
                a("dir", "."),
                a("statsFile", null)
        );
        // task
        CriticalityEvaluator criticalityEvaluator = new CriticalityEvaluator(
                finalT, // task duration
                new Settings() // default settings for the physics engine
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

            int bodySize = (int) body.values().stream().filter(Objects::nonNull).count();
            if (bodySize < 2) {
                return 0.0;
            }
            double[] avalanchesSpatialExtension = new double[bodySize + 1];
            double[] avalanchesTemporalExtension = new double[100];

            // a pulse controller is applied on each voxel
            body.stream()
                .filter(Objects::nonNull)
                .forEach(voxel -> {
                    Controller<ControllableVoxel> pulseController = new TimeFunctions(Grid.create(body.getW(), body.getH(), (x, y) -> (Double t) -> {
                    if (x == voxel.getX() && y == voxel.getY()) {
                        if (t < pulseDuration/2) {
                            return 1.0;
                        } else if (t < pulseDuration) {
                            return -1.0;
                        }
                    }
                    return 0.0;
                }));
                List<Double> metrics = task.apply(new Robot<>(pulseController, SerializationUtils.clone(body)));
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
            if (spatialSizeNumber < 2 || temporalSizeNumber < 2) {
                return 0.0;
            }
            // compute the log-log of the 2 distributions
            List<Point2> logLogSpatialDistribution = IntStream.range(1, avalanchesSpatialExtension.length)
                    .mapToObj(i -> Point2.build(Math.log10(i), avalanchesSpatialExtension[i] > 0.0 ? Math.log10(avalanchesSpatialExtension[i]) : 0))
                    .collect(Collectors.toList());

            List<Point2> logLogTemporalDistribution = IntStream.range(1, avalanchesTemporalExtension.length)
                    .mapToObj(i -> Point2.build(Math.log10((i+1) * binSize), avalanchesTemporalExtension[i] > 0.0 ? Math.log10(avalanchesTemporalExtension[i]) : 0))
                    .collect(Collectors.toList());

            // linear regression of the log-log distribution
            LinearRegression spatialLinearRegression = new LinearRegression(logLogSpatialDistribution);
            LinearRegression temporalLinearRegression = new LinearRegression(logLogTemporalDistribution);
            double RSquared = 0;
            if (!Double.isNaN(spatialLinearRegression.R2())) {
                RSquared = spatialLinearRegression.R2(); //(spatialLinearRegression.R2() + temporalLinearRegression.R2())/2;
            }
            // 3. KS statistics
            double ks1 = computeKSStatistics(logLogSpatialDistribution, spatialLinearRegression);
            double ks2 = computeKSStatistics(logLogTemporalDistribution, temporalLinearRegression);
            double DSquared = Math.pow(Math.exp(-(0.9 * Math.min(ks1, ks1) + 0.1 * (ks1 + ks1)/2)), 2d); //Math.pow(Math.exp(-(0.9 * Math.min(ks1, ks2) + 0.1 * (ks1 + ks2)/2)), 2d);
            return RSquared + DSquared;
        };

        // direct mapper
        Function<List<Double>, Grid<ControllableVoxel>> directMapper = g -> {
            Grid<ControllableVoxel> body = null;
            // ordered grid
            List<Double> thresholds = g.stream().distinct().sorted().collect(Collectors.toList());
            Collections.reverse(thresholds);
            for (double directThreshold : thresholds) {
                // build grid with fixed number of voxels and dynamic threshold
                body = Utils.gridLargestConnected(Grid.create(gridSide, gridSide, (x, y) -> g.get(gridSide * x + y) > directThreshold ? SerializationUtils.clone(softMaterial) : null), Objects::nonNull);
                // find largest connected and crop
                body = Utils.gridLargestConnected(body, Objects::nonNull);
                body = Utils.cropGrid(body, Objects::nonNull);
                if (body.values().stream().filter(Objects::nonNull).count() >= robotVoxels) {
                    break;
                }
            }
            return body;
        };

        // gaussian mapper
        Function<List<Double>, Grid<ControllableVoxel>> gaussianMapper = g -> {
            Grid<Double> gaussianGrid = Grid.create(gridSide, gridSide, 0d);
            double epsilon = 0.0001;
            int c = 0;
            for (int j = 0; j < nGaussian; j++) {
                // extract parameter of the j-th gaussian for the i-th material
                double muX = g.get(c + 0);
                double muY = g.get(c + 1);
                double sigmaX = Math.max(0d, g.get(c + 2)) + epsilon;
                double sigmaY =  Math.max(0d, g.get(c + 3)) + epsilon;
                double weight = g.get(c + 4) * 2d -1;
                c = c + 5;
                // compute over grid
                for (int ix = 0; ix < gridSide; ix++) {
                    for (int iy = 0; iy < gridSide; iy++) {
                        double x = (double)ix/(double)gridSide;
                        double y = (double)iy/(double)gridSide;
                        gaussianGrid.set(ix, iy, gaussianGrid.get(ix, iy) + weight * Math.exp(-(Math.pow(x - muX, 2d)/(2.0 * Math.pow(sigmaX, 2d)) + Math.pow(y - muY, 2d)/(2.0 * Math.pow(sigmaY, 2d)))));
                    }
                }
            }
            Grid<ControllableVoxel> body = null;
            // ordered grid
            List<Double> thresholds = gaussianGrid.values().stream().distinct().sorted().collect(Collectors.toList());
            Collections.reverse(thresholds);
            for (double gaussianThreshold : thresholds) {
                // build grid with fixed number of voxels and dynamic threshold
                body = Grid.create(gridSide, gridSide, (x, y) -> gaussianGrid.get(x, y) > gaussianThreshold ? SerializationUtils.clone(softMaterial) : null);
                // find largest connected and crop
                body = Utils.gridLargestConnected(body, Objects::nonNull);
                body = Utils.cropGrid(body, Objects::nonNull);
                if (body.values().stream().filter(Objects::nonNull).count() >= robotVoxels) {
                    break;
                }
            }
            return body;
        };

        // old evolver
        Evolver<List<Double>, Grid<ControllableVoxel>, Double> directEvolver = new StandardEvolver<>(
                directMapper,
                new FixedLengthListFactory<>(gridSide * gridSide, new UniformDoubleFactory(0, 1)),
                PartialComparator.from(Double.class).reversed().comparing(Individual::getFitness), // fitness comparator
                popSize, // pop size
                Map.of(
                        new GaussianMutation(mutationProb), 0.2d,
                        new UniformCrossover<>(new FixedLengthListFactory<>(gridSide * gridSide, new UniformDoubleFactory(0, 1))), 0.8d
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
                        new Item("body", bodyToString(i.getSolution()), "%s")
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
                        new Births(birth),
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