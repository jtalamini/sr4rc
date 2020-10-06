package it.units.erallab;

import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.core.controllers.Controller;
import it.units.erallab.hmsrobots.core.controllers.TimeFunctions;
import it.units.erallab.hmsrobots.core.objects.ControllableVoxel;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Point2;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.Problem;
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
import it.units.malelab.jgea.representation.sequence.numeric.GaussianMutation;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;
import org.dyn4j.dynamics.Settings;
import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.function.Function;

public class ControllerOptimization extends Worker {

    public ControllerOptimization(String[] args) {
        super(args);
    }

    private static Grid<ControllableVoxel> getRandomNeighbour(Grid<ControllableVoxel> body, int x, int y, int numberOfVoxels, Random random) {
        if (body.values().stream().filter(Objects::nonNull).count() == numberOfVoxels) {
            return body;
        } else {
            // set voxel
            body.set(x, y, new ControllableVoxel());
            List<Point2> neighbours = new ArrayList<>();

            if (x-1 >= 0) {
                neighbours.add(Point2.build(x-1,y));
            }
            if (x+1 < body.getW()) {
                neighbours.add(Point2.build(x+1, y));
            }
            if (y-1 >= 0) {
                neighbours.add(Point2.build(x, y-1));
            }
            if (y+1 < body.getH()) {
                neighbours.add(Point2.build(x, y+1));
            }
            Point2 randomNeighbour = neighbours.get(random.nextInt(neighbours.size()));
            return getRandomNeighbour(body, (int)randomNeighbour.x, (int)randomNeighbour.y, numberOfVoxels, random);
        }
    }

    private static Grid<ControllableVoxel> generateRandomBody(int numberOfVoxels, int gridSide, Random random) {
        Grid<ControllableVoxel> body = Grid.create(gridSide, gridSide);
        return getRandomNeighbour(body, random.nextInt(gridSide), random.nextInt(gridSide), numberOfVoxels, random);
    }

    private static void printBody(Grid<ControllableVoxel> body) {
        System.out.println("----BEGIN BODY----");
        for (int y = 0; y < body.getH(); y++) {
            for (int x = 0; x < body.getW(); x++) {
                if (body.get(x, y) != null) {
                    System.out.print("#");
                } else {
                    System.out.print(" ");
                }
            }
            System.out.println();
        }
    }

    public static void main(String[] args) {
        new SR4RC(args);
    }

    public void run() {

        int gridSide = 10;
        int numberOfVoxels = 20;
        int randomSeed = 1;
        int cacheSize = 10000;

        Random random = new Random(randomSeed);

        int popSize = 20;
        int iterations = 100;
        double mutationProb = 0.001;
        int tournamentSize = 10;

        /*
        int counter = 0;
        while (counter < 10) {
            Grid<ControllableVoxel> randomBody = generateRandomBody(numberOfVoxels, gridSide, random);
            printBody(randomBody);
            counter += 1;
        }
         */

        String serializedSolution = "H4sIAAAAAAAAAOxUv28TMRR+vbYktGppmoIYYCIsSM0NlZBQhjTKXUqka4tyEYJ2KG7iJK58d8Hna68dKvEHsFMEGwMDKwMSLCxFTNCO/AdISF3Y8bv8aAAJFSSkDGfp7OfPz/6+9/x8r77BuC8gw2Q2cJn0s1QQzslmtuX4wtv0FBJIxrNLgtXfn792qXDz8LUGo2UYaalvxwJN+hJmrC2yTXT01C3my1zYBoCE+s6pw9O42DmmIATZRY/w0dHVg0PyfBRGyjDmsz0abbmxM4Z9WzW18dYfVNU8QbPe5hatqUnRc6Xw0IPTu15I+fdnGx+ul29f0UAzIOmQsOSJGrVgsoHjMpUtry5hyWJSjwj0LoHeJ9CRQO8S6L8RZEqnB+VCAfNnlRrtfpcIX5w8WH2rwZQBKSIoqRDJvGUSGpRLoiQ3BKspxF2DOc4cJiMBJIJKnDQNGHOI7xswi0PBbQacCIM4beY21yCFYFHJZX7fP8Is5tK+nwFpxGxWpxZ1m7IVaTBgUlBfMhkglwETfn/dgITfFrizb5UsmO1Ydo00Gh6vK1MVRHqgIEw3cGyKNZH/BFFbvN8ZR/InHaM39o388ULU8m8OsD1dTP6ML8515li6p9XVZcrYVDDC2R7G494RXrg7njj+sl39+FnVgyoCyqlDXVndbVMJqY5UTtymXuQqIbl1SHY9VCSp9YF1JFBxbAvI/9V9Z+xfc9SNF2YBVPVMRzEgRRTD4GI7ED0Npw65l4+/zi8c6U809FCe2v5D2IdpCVN22TA3zHtVs7JSsPpo0VypVgrWRrGyatu99O8LMP/5hQ0+gEG9yHhBQtIo29XCStHEl4zY+H+4fDx3AruZAPsUJgKNNHYXe0Q4uTwcIuJcDFsuYg2xhlhDrGHIf1OxCBiioog19B5H+AMAAP//AwDdV83KQA4AAA==";
        Grid<ControllableVoxel> body = it.units.erallab.Utils.safelyDeserialize(serializedSolution, Grid.class);
        printBody(body);

        MultiFileListenerFactory<Object, Controller<ControllableVoxel>, Double> statsListenerFactory = new MultiFileListenerFactory<>(
                a("dir", "."),
                a("statsFile", null)
        );

        final Locomotion locomotion = new Locomotion(
                20,
                Locomotion.createTerrain("flat"),
                Lists.newArrayList(Locomotion.Metric.TRAVELED_X_DISTANCE),
                new Settings()
        );
        Function<Robot<?>, List<Double>> task = Misc.cached(locomotion, 10000);

        Problem<Controller<ControllableVoxel>, Double> problem = () -> brain -> {
            Robot<ControllableVoxel> robot = new Robot<>(brain, body);
            List<Double> results = task.apply(robot);
            return results.get(0);
        };

        Function<List<Double>, Controller<ControllableVoxel>> mapper = g -> {
            // each element of g becomes a phase
            Controller<ControllableVoxel> brain = new TimeFunctions(
                    Grid.create(gridSide, gridSide, (x, y) -> (Double t) -> Math.sin(-2 * Math.PI * t + Math.PI * g.get(x + y * gridSide)))
            );
            return brain;
        };

        // evolver
        Evolver<List<Double>, Controller<ControllableVoxel>, Double> evolver = new StandardEvolver<>(
                mapper,
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

        List<DataCollector<?, ? super Controller<ControllableVoxel>, ? super Double>> collectors = List.of(
                new Basic(),
                new Population(),
                new Diversity(),
                new BestInfo("%6.4f"),
                new FunctionOfOneBest<>(i -> List.of(
                        new Item("serialized.grid", it.units.erallab.Utils.safelySerialize(i.getSolution()), "%s")
                ))
        );
        Listener<? super Object, ? super Controller<ControllableVoxel>, ? super Double> listener;
        if (statsListenerFactory.getBaseFileName() == null) {
            listener = listener(collectors.toArray(DataCollector[]::new));
        } else {
            listener = statsListenerFactory.build(collectors.toArray(DataCollector[]::new));
        }
        try {
            evolver.solve(
                    Misc.cached(problem.getFitnessFunction(), cacheSize),
                    new Iterations(iterations),
                    new Random(randomSeed),
                    executorService,
                    listener
            );
        } catch (InterruptedException e) {
            e.printStackTrace();
        } catch (ExecutionException e) {
            e.printStackTrace();
        }

    }
}
