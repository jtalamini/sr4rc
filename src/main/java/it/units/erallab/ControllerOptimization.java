package it.units.erallab;

import it.units.erallab.hmsrobots.core.controllers.CentralizedSensing;
import it.units.erallab.hmsrobots.core.controllers.Controller;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.controllers.TimeFunctions;
import it.units.erallab.hmsrobots.core.objects.ControllableVoxel;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.objects.Voxel;
import it.units.erallab.hmsrobots.core.sensors.*;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Point2;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.Problem;
import it.units.malelab.jgea.core.evolver.CMAESEvolver;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.stopcondition.Births;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.core.listener.MultiFileListenerFactory;
import it.units.malelab.jgea.core.listener.collector.*;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.util.Misc;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;
import org.apache.commons.lang3.SerializationUtils;

import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.function.Function;
import java.util.stream.IntStream;

import static it.units.malelab.jgea.core.util.Args.i;

public class ControllerOptimization extends Worker {

    public ControllerOptimization(String[] args) {
        super(args);
    }

    private static Grid<ControllableVoxel> getRandomNeighbour(Grid<ControllableVoxel> body, int x, int y, int numberOfVoxels, Random random, ControllableVoxel material) {
        if (body.values().stream().filter(Objects::nonNull).count() == numberOfVoxels) {
            return body;
        } else {
            // set voxel
            body.set(x, y, SerializationUtils.clone(material));
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
            return getRandomNeighbour(body, (int)randomNeighbour.x, (int)randomNeighbour.y, numberOfVoxels, random, material);
        }
    }

    private static Grid<ControllableVoxel> generateRandomBody(int numberOfVoxels, int gridSide, Random random, ControllableVoxel material) {
        Grid<ControllableVoxel> body = Grid.create(gridSide, gridSide);
        return getRandomNeighbour(body, random.nextInt(gridSide), random.nextInt(gridSide), numberOfVoxels, random, material);
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
        new ControllerOptimization(args);
    }

    public void run() {
        String bodyType = a("bodyType", "serialized");
        String controller = a("controller", "centralized");
        int robotIndex = i(a("robotIndex", "0"));
        int gridW = i(a("gridW", "5"));
        int gridH = i(a("gridH", "4"));
        int robotVoxels = i(a("robotVoxels", "20"));
        int randomSeed = i(a("randomSeed", "666"));
        Random random = new Random(randomSeed);
        int cacheSize = 10000;
        int births = i(a("births", "5000"));
        String taskType = a("taskType", "locomotion");

        Grid<ControllableVoxel> body;

        if (bodyType.equals("random")) {
            body = generateRandomBody(robotVoxels, Math.max(gridW, gridH), random, Material.softMaterial);
        } else if (bodyType.equals("box")) {
            body = Grid.create(gridW, gridH, (x,y) -> SerializationUtils.clone(Material.softMaterial));
        } else {
            List<String> bodies = List.of(
                    "H4sIAAAAAAAAAOxUv28TMRR+vbYktGppmoIYYCIsSM0NlZBQhjTKXUqka4tyEYJ2KG7iJK58d8Hna68dKvEHsFMEGwMDKwMSLCxFTNCO/AdISF3Y8bv8aAAJFSSkDGfp7OfPz/6+9/x8r77BuC8gw2Q2cJn0s1QQzslmtuX4wtv0FBJIxrNLgtXfn792qXDz8LUGo2UYaalvxwJN+hJmrC2yTXT01C3my1zYBoCE+s6pw9O42DmmIATZRY/w0dHVg0PyfBRGyjDmsz0abbmxM4Z9WzW18dYfVNU8QbPe5hatqUnRc6Xw0IPTu15I+fdnGx+ul29f0UAzIOmQsOSJGrVgsoHjMpUtry5hyWJSjwj0LoHeJ9CRQO8S6L8RZEqnB+VCAfNnlRrtfpcIX5w8WH2rwZQBKSIoqRDJvGUSGpRLoiQ3BKspxF2DOc4cJiMBJIJKnDQNGHOI7xswi0PBbQacCIM4beY21yCFYFHJZX7fP8Is5tK+nwFpxGxWpxZ1m7IVaTBgUlBfMhkglwETfn/dgITfFrizb5UsmO1Ydo00Gh6vK1MVRHqgIEw3cGyKNZH/BFFbvN8ZR/InHaM39o388ULU8m8OsD1dTP6ML8515li6p9XVZcrYVDDC2R7G494RXrg7njj+sl39+FnVgyoCyqlDXVndbVMJqY5UTtymXuQqIbl1SHY9VCSp9YF1JFBxbAvI/9V9Z+xfc9SNF2YBVPVMRzEgRRTD4GI7ED0Npw65l4+/zi8c6U809FCe2v5D2IdpCVN22TA3zHtVs7JSsPpo0VypVgrWRrGyatu99O8LMP/5hQ0+gEG9yHhBQtIo29XCStHEl4zY+H+4fDx3AruZAPsUJgKNNHYXe0Q4uTwcIuJcDFsuYg2xhlhDrGHIf1OxCBiioog19B5H+AMAAP//AwDdV83KQA4AAA==",
                    "H4sIAAAAAAAAAOxUz2sTQRR+3bYmtrQ2TRUPejIemxUKguSQhuymBratZINoe6jTZJJM2V/OzrbbHgr+Ad6t6M2DB68eBL14qXjS9uh/IAi9eHfeJtlGhaKCsIcM7MybN2/m+96bb/bVNxj3OeSYyAcOE36ecmJZZDPfsX3ubrrSEwhm5Zc4a74/f+1S6ebhawVGqzDSkd+OAYrwBcwYW2SbqBipGswXhdADgLT8zsnDs7jYPabEOdnFiPDR0dWDQ/J8FEaqMOazPRptubEzhr0nd906g1LD5TTvbm7RhpyUXUdwFyMsetcNqfX92caH69XbVxRQNEjbJKy4vEENmGzhuExFx20KWDKYUCMAtQegxgAqAqg9APU3gFzl9KBCyGH+T6lGu9+lwhcnD1bfKjClQYZwSmpEMHeZhBq1BJGUW5w1pMdZgzmL2UxEBEjkqlikrcGYTXxfg1kcSk47sAjXiO0xp70GGXSWJV3mx/GRz2AOjeM0yKLPZE1qUKctOhEHDSY59QUTAWJpMOHH6xqkfI/jztiqGDDbtcwGabVcqylNqYbsgBp0J7BNioIofoKoLd7vjiPFk67RH2OjeLwQteKbA2xPF9M/+xfnunPU7am0ekg5k3JGLLaH+Th3uBvujqeOv2zXP36WepAioBa1qSPqux4VkOlStYjTVsuWLEhhHdK9CJlJZn1gHQFkHtscin913znz1xr18oVZAKme6SgHhIhyGFz0At7ncBpQePn46/zCkfpEwQgZqew/hH2YFjBlVjV9Q79X12srJSP2lvWVeq1kbJRrq6bZL/8+B/2fX9jgAxjki4gXBKS1qlkvrZR1Hx3j/+Hm8dwJ7GYC7DNYBTSy2F3sA+Hk8pBDcjh4XjJYeAkhkggSyajFkEOCNDHkMOQw/EcMOZzBIRmKSAqLhDyQ8AcAAAD//wMAaSv5Js8PAAA=",
                    "H4sIAAAAAAAAAOxTv28TMRR+vbZqaNXSNAUxwEQYmxsqIVCGNMpdSqRri3IRgnYobuIkrnw/8Pnaa4dK/AHsFMHGwMDKgAQLSxETtCP/ARJSF3b8Lj8aQEKAhHRDnmT7+fnZ3/fszy+/wnggIMtkLnSZDHJUEM7JVq7tBMLb8lQklIznlgVrvDt39WLx+tErDUYrMNJWbdcCTQYSZq1tskN0zNQtFsh85ANASrUJdXgGFzvHFIUge5gRPTy+cnhEno3CSAXGArZP4y03dsew95WpjTd/w6ruCZrztrZpXU1KniuFhxmc3vEiyr893Xx/rXLrsgaaASmHRGVP1KkFU00cV6hsew0JyxaTegygdwH0PoCOAHoXQP8FIFs+OygfCVj4U6rx7rcT0fPT+2tvNJg2IE0EJVUimbdCIoNySRTlpmB1FXHXYZ4zh8mYAIlDZU5aBow5JAgMmMOh6LZCToRBHJ+5rXVIY7Ck6LKgnx/HLObSfp4BGYzZrEEt6rZkO+ZgwJSggWQyRCwDJoP+uqFe0xe4s++VLZjreHadNJsebyhXCSIzIAjTDR2boiYKHyG2pXudcaRw2nF6Y98pnCzGVnh9iPZkKfVjfGm+M0fpnqmri5S1qWCEs32sx70tvGhvfOLk807twyelByUCyqlDXVnb86mEdIcqJ25LL3F1IfkNSHUzVCXpjYF1BFB17Ago/NV7Z+2f76hbL8wBKPXMxDUgRFzD4KIfih6Hs4T8i0dfFhaP9ccaZqhM7eABHMCMhGm7Ypib5t2aWV0tWv1oyVytVYvWZqm6Ztu96z8QYP7zDxv8AIN8EfG8hJRRsWvF1ZLpxxZgePw/vD+eO4ndbIh9Gu8CnQx2F3pAOLmUGCJDDsnhkBRNJOqbJIHGkENyOCRFmkMOyeEwFESCOCTiMXw/+g4AAP//AwANM3BBTg4AAA==",
                    "H4sIAAAAAAAAAOxVv2/TQBR+dVsaWrU0TUEMMBFGYqRKSChDGsVOieS2KI4QtEO5JpfkqvMPzufW7VCJP4CdItgYGFgZkGBhKWKCduQ/QELqws4950cDSAiQkDzkJN+9++7dfd9792y//ArjgYAsk7nQZTLIUUE4J5u5thMIb9NTSCgZzy0J1nh39sqF4o3DVxqMVmCkrZ4dCzQZSJi1tsg20dFTt1gg85EPACn1nFGHZ3Cxc0xRCLKLHtHDo8sHh+TZKIxUYCxgezTecn1nDHvf99W+m78RVfcEzXmbW7SuJiXPlcJDD07veBHl355uvL9auXVJA82AlEOisifq1IKpJo7LVLa9hoQli0k9JtC7BHqfQEcCvUug/0KQLZ8elI8EXPtTqfHutxPR85P7q280mDYgTQQlVSKZt0wig3JJlOSmYHWFuGswz5nDZCyAxFCZk5YBYw4JAgPmcCi6rZATYRDHZ25rDdIIlpRcFvT9Y8xiLu37GZBBzGYNalG3JduxBgOmBA0kkyFyGTAZ9NcNmAh8gTv7VtmCuY5l10mz6fGGMlU9ZAbqwXRDx6ZYEoWPELfFe51xpHDSMXpj3ygcL8St8PoA25PF1I/44nxnjpV7WlxdpqxNBSOc7WE87m3hRbvjE8eft2sfPql6UEVAOXWoK2u7PpWQ7kjlxG3pJa4Skl+HVNdDRZJeH1hHAhXHtoDCX9131v45R914YQ5AVc9MHANSxDEMLvqh6Gk4dci/ePTl2sKR/lhDD+Wp7T+AfZiRMG1XDHPDvFszqytFq4+WzJVatWhtlKqrtt1L/74A85/fsMEXYFAvMp6TkDIqdq24UjL9AJHx/3D1eO4kdrMh9mlMAxoZ7M73iHByET8nSZCRCBHJyMVQw1DDUENivxFJUDHUkBwNCflrJENEQnLhJ0ZI9B0AAP//AwDIMfUmRg4AAA==",
                    "H4sIAAAAAAAAAOxUv28TMRR+vbYktGppmoIYYCIsSM0NlZBQhjTKXUqka4tyEYJ2KG7iJK7uFz5fe+1QiT+AnSLYGBhYGZBgYSlignbkP0BC6sKO3yW5BpBQQULKcJbOfv787O97z8/36huM+xxyTOQDhwk/TzmxLLKZ79g+dzddiQSCWfklzprvz1+7VLp5+FqB0SqMdOS3Y4AifAEzxhbZJip6qgbzRSH0AOCc/FLy8Cwudo8pcU520SN8dHT14JA8H4WRKoz5bI9GW27sjGEvN936g6KGy2ne3dyiDTkpu47gLnpY9K4bUuv7s40P16u3ryigaJC2SVhxeYMaMNnCcZmKjtsUsGQwoUYEao9AjQlUJFB7BOpvBLnK6UGFkMP8WaVGu9+lwhcnD1bfKjClQYZwSmpEMHeZhBq1BJGSW5w1JOKswZzFbCYiASSCKhZpazBmE9/XYBaHktMOLMI1YnvMaa9BBsGylMv82D/CDObQ2E+DLGIma1KDOm3RiTRoMMmpL5gIkEuDCT9e1+RNehx3xlbFgNmuZTZIq+VaTWnKYsgOFIPuBLZJsR6KnyBqi/e740jxpGv0x9goHi9ErfjmANvTxfTP+OJcd45le1pZPaacSTkjFtvDeJw73A13x1PHX7brHz/LepBFQC1qU0fUdz0qINOVahGnrZYtmZDCOqR7HjKSzPrAOhLIOLY5FP/qvnPmrznqxQuzALJ6pqMYkCKKYXDRC3hfw6lD4eXjr/MLR+oTBT2kp7L/EPZhWsCUWdX0Df1eXa+tlIwYLesr9VrJ2CjXVk2zn/59Dvo/v7DBBzCoFxkvCEhrVbNeWinrnucjNP4f7h7PncBuJsA+g3lAI4vdxT4RTi57wyJjOFQMhYxEw/Bo8LykKBINiYZEQ6LhbD/L5G/Zz0b4AwAA//8DAAAaRYZADgAA",
                    "H4sIAAAAAAAAAOyUz2sTQRTHX7etiS2tTVPFg56Mx2YPBUEipCG7qYFtK9kg2h7qNJkkU/aXs7PttoeCf4B3K3rz4MGrB0EvXiqetD36HwhCL96dt0m2UUFUEFbIwM68ffNmvp8383ZffIFxn0OOiXzgMOHnKSeWRTbzHdvn7qYrPYFgVn6Js+bbs1culK4dvlRgtAojHfnsGKAIX8CMsUW2iYqRqsF8UQg9ADgjnwm5eRYnu9uUOCe7GBE+OLp8cEiejsJIFcZ8tkejJTd2xrD3PE+uu/4LqIbLad7d3KIN+VJ2HcFdjLDobTek1tcnG++uVm9eUkDRIG2TsOLyBjVgsoXjMhUdtylgyWBCjQTUnoAaC6gooPYE1J8EcpXTjQohh/nfRY1Wv0mFz07urb5WYEqDDOGU1Ihg7jIJNWoJIpFbnDWkx1mDOYvZTEQAJHJVLNLWYMwmvq/BLA4lpx1YhGvE9pjTXoMMOssSl/lxfOQzmEPjOA2y6DNZkxrUaYtOxKDBJKe+YCJALU3eXzyvQcr3OK6MrYoBs13LbJBWy7Wa0pT1kB2oB90JbJNiSRQ/QNQW73bHkeJJ1+iPsVE8Xoha8dUBtseL6e/9i3Pdd6zc0+LqKeVMyhmx2B7m49zibrg7njr+tF1//1HWgywCalGbOqK+61EBmS6qRZy2WrbkgRTWId2LkJlk1gfmUUDmsc2h+Ef3nTN/PKNevjALIKtnOsoBJaIcBie9gPcZTgMKzx9+nl84Uh8pGCEjlf37sA/TAqbMqqZv6Hfqem2lZMTesr5Sr5WMjXJt1TT7x7/PQf/rL2zwAxjkRcVzAtJa1ayXVsq6128+zoz/gxLAfSewmwmwz+BxoJHF7nxfCF8uJgcjMSAJwUgGSBIYknESwysZMgwZhgxDhv+EIRE/62RAeOE3AAAA//8DAF/MROPVDgAA",
                    "H4sIAAAAAAAAAOxUv28TMRR+vbZqaNVCmoIYYCKMzQ2VkFCGJMpdSqRri3IRgnYobuIkrnw/8Pnaa4dK/AHsFMHGwMDKgAQLSxETtCP/ARJSF3b8Lj8aqECAhNThLJ39/PnZ3/een+/lVxgPBGSZzIUuk0GOCsI52ch1nEB4G55CQsl4blGw5rtz1y6Vbhy80mC0CiMd9W1boMlAwgVrk2wRHT11iwUyH/kAkMJPHZ7Bxe4xJSHIDnpEDw+v7h+QZ6MwUoWxgO3SeEtxewx731fbbv5GU8MTNOdtbNKGmpQ9VwoPPTi940WUf3u6/v569dYVDTQDUg6JKp5oUAumWjguUdnxmhIWLSb1mEDvEegDAh0J9B6BfoogWzk5KB8JmP9TqfHutxPR8+P7K280mDYgTQQlNSKZt0Qig3JJlOSWYA2FuKswx5nDZCyAxFCFk7YBYw4JAgNmcSi57ZATYRDHZ257FdIIlpVcFgz8Y8xiLh34GZBBzGZNalG3LTuxBgOmBA0kkyFyGTAZDNYNmAh8gTsHVsWC2a5lN0ir5fGmMlU5ZIbKwXRDx6ZYEYWPELfive44UjjuGv1xYBSOFuJWeL2P7Ukx9SNenOvOsXBPaqvHlLWpYISzXYzHvS28aGd84ujzVv3DJ1UPqggopw51ZX3HpxLSXamcuG29zFVC8muQ6nmoSNJrQ+tIoOLYElD4q/vO2j/nqBcvzAKo6pmJY0CKOIbhRT8UfQ0nDvkXj77MLxzqjzX0UJ7a3gPYgxkJ03bVMNfNu3WztlyyBmjZXK7XStZ6ubZi2/307wkw//mFDT+AYb3IeF5Cyqja9dJy2fSxBYiO/4frx3MnsbsQYp/GVKCRwe5inwgnlxMNiYZEw2kNyfMcTkWSjURDouEs/yj8s/NKEym/khJ9BwAA//8DANUorm1WDgAA",
                    "H4sIAAAAAAAAAOxVv2/TQBR+TVs1tGppkoIYYCKMjYdKSChDEsVOieS2KI4QtEO5JpfkKv/ifG7dDpX4A9gpgo2BgZUBCRaWIiZoR/4DJKQu7NyzEzeAVAESkoec5Lt3776773vvnu1X32DS45BnouDbTHgFyolpkq1Cz/K4s+VIjy+YWVjmrP3+wvXLlZtHr1MwXoexnnx2dUgJT8C8vk12iIJIRWeeKAYuAKTxkYfncDE6psI52UNE8Oj42uEReT4OY3WY8Ng+DbeUdyewd11X7rt1jqiWw2nB2dqmLTmpOrbgDiJMetcJqPn92eaHG/XbV1OQUiFtkaDm8BbVYaaD4woVPactYFlnQgkJlD6BEhMoSKD0CZTfCPK1s4OKAYfFP5Ua7n43Fbw4fbD2NgWzKmQIp6RBBHNWSKBSUxApucNZS3rsdVgwmcVEKICErppJuipMWMTzVMjiULG7vkm4SiyX2d11yKCzKuUyL8aHPp3ZNMapkEOfwdpUp3ZX9EINKsxw6gkmfORSYdqL11WY8lyOO2OrpkM2sowW6XQcsy1NWQ+5oXrQbN8yKJZE6ROErXw/GsdKp5ExGGOjdLIUttKbQ2xPy+mf/eWFaI6Ve1Zcfaa8QTkjJtvHeOw73An2JqdOvuw0P36W9SCLgJrUorZo7rlUQCaSahK7q1RNmZDiBqT7CBlJZmNoHQlkHDscSn9133nj1xz144UsgKyeuTAGpAhjGF50fT7QcAYovnz8dXHpWHmSQoREpg4ewgHMCZg16qq2qd1rao3Vih57q9pqs1HRN6uNNcMYpP+Ag/bPb9jwCzCsFxkvCkirdaNZWa1qbtQ89E/+hwLAc6exm/exz2Ay0Mhhd2lAhJMrSdCQmGQkQkQSNCTjOkYaRhpGGpKpIRlfiGSIGP2+Rtk4LxsJyYfrBj8AAAD//wMA8rPNMtkOAAA=",
                    "H4sIAAAAAAAAAOyWv28TMRTHX6+tGlq1NE1BDDARxuaGSgiUIY1ylxLp2qJchKAdips4iSvfD3y+9tqhEn8AO0WwMTCwMiDBwlLEBO3If4CE1IUdv8uPBpAQICHdEEtnv3t+9vfj53dRXn6F8UBAlslc6DIZ5KggnJOtXNsJhLflKU8oGc8tC9Z4d+7qxeL1o1cajFZgpK2eXQs0GUiYtbbJDtExUrdYIPORDwAp9UyozTM42dmmKATZw4jo4fGVwyPybBRGKjAWsH0aL7mxO4a97/tq3c3fQNU9QXPe1jatq5eS50rhYQSnd7yI8m9PN99fq9y6rIFmQMohUdkTdWrBVBPHFSrbXkPCssWkHgvoXQG9L6CjgN4V0H8RyJbPNspHAhb+FDVe/XYien56f+2NBtMGpImgpEok81ZIZFAuiUJuClZXHncd5jlzmIwBSOwqc9IyYMwhQWDAHA5FtxVyIgzi+MxtrUManSWFy4J+fOyzmEv7cQZk0GezBrWo25LtmMGAKUEDyWSIWgZMBv15Q12mL3Bl3ypbMNex7DppNj3eUKaqh8xAPZhu6NgUS6LwEeK2dK8zjhROO0Zv7BuFk8W4FV4fYnuylPrRvzTfecfKPSuurlLWpoIRzvbxPO5t4UV74xMnn3dqHz6pelBFQDl1qCtrez6VkO6gcuK29BJXCclvQKoboU6S3hiYRwF1jh0Bhb+676z9c46654U5AFU9M/EZUCI+w+CkH4oew1lA/sWjLwuLx/pjDSNUpHbwAA5gRsK0XTHMTfNuzayuFq2+t2Su1qpFa7NUXbPtXvoPBJj//IUNfgCDvKh4XkLKqNi14mrJ9OMWoHv8P9w/7juJ3WyIfRpzgUYGuws9IXy5lBSKRGAkgWGYiCHDkGHIkPRf7ERAJIJiyJAchgT9r0oKSRIYVCqi7wAAAP//AwBNUpWXVA8AAA==",
                    "H4sIAAAAAAAAAOyVz2sTQRTHX7etjS2tTVPFg56MR7NCQZAc0pDd1MC2lWwQbQ91mkySKbM/nJ1ttz0U/AO8W9GbBw9ePQh68VLxpO3R/0AQevHuvE2yjQqigpBDBnbm7Zs38/28mbfJy68wHgjIMpkLXSaDHBWEc7KZazuB8DY95Qkl47klwRrvzl65ULxx+EqD0QqMtNWzY4EmAwmz1hbZJjpG6hYLZD7yAeCMelJq8wxOdrYpCkF2MSJ6eHT54JA8G4WRCowFbI/GS67vjGHv+2rZzd8w1T1Bc97mFq2rl5LnSuFhBKd3vIjyb0833l+t3LqkgWZAyiFR2RN1asFUE8dlKtteQ8KSxaQeC+hdAT0R0FFA7wrovwhky6cb5SMB1/4UNV79diJ6fnJ/9Y0G0wakiaCkSiTzlklkUC6JQm4KVlcedw3mOXOYjAFI7Cpz0jJgzCFBYMAcDkW3FXIiDOL4zG2tQRqdJYXLgiQ+9lnMpUmcARn02axBLeq2ZDtmMGBK0EAyGaKWAZNBMm/AROALXJlYZQvmOpZdJ82mxxvKVOWQ6SsH0w0dm2JFFD5C3BbvdcaRwknH6I2JUTheiFvh9QG2J4upH/2L8513LNzT2uoqZW0qGOFsD/Nxbwsv2h2fOP68XfvwSdWDKgLKqUNdWdv1qYR0B5UTt6WXuDqQ/DqkuhEqk/R63zwKqDy2BRT+6r6z9s9n1M0X5gBU9czEOaBEnEP/pB+KHsNpQP7Foy/XFo70xxpGqEht/wHsw4yEabtimBvm3ZpZXSlaibdkrtSqRWujVF217d7x7wsw//kL6/8A+nlR8ZyElFGxa8WVkuljC9A7/h+uH/edxG42xD6NR4FGBrvzPSF8uTiESCCGF5JADBmGDEOGIcMgMwzED+WA/GcMBsVAQAwIRdyi7wAAAP//AwApet0cRg4AAA=="
            );
            body = it.units.erallab.Utils.safelyDeserialize(bodies.get(robotIndex), Grid.class);
        }

        MultiFileListenerFactory<Object, Robot<? extends Voxel>, Double> statsListenerFactory = new MultiFileListenerFactory<>(
                a("dir", "."),
                a("statsFile", null)
        );

        Function<?, ?> task = null;

        if (taskType.equals("locomotion")) {
            task = Misc.cached(ControllerOptimizationTask.locomotion, 10000);
        } else if (taskType.equals("hiking")) {
            task = Misc.cached(ControllerOptimizationTask.hiking, 10000);
        } else if (taskType.equals("stairway")) {
            task = Misc.cached(ControllerOptimizationTask.stairway, 10000);
        } else if (taskType.equals("jump")) {
            task = Misc.cached(ControllerOptimizationTask.jump, 10000);
        }

        Function<Robot<? extends Voxel>, ?> finalTask = (Function<Robot<? extends Voxel>, ?>) task;
        Problem<Robot<? extends Voxel>, Double> problem = () -> robot -> {
            List<Double> results = (List<Double>) finalTask.apply(robot);
            return results.get(0);
        };

        Function<List<Double>, Robot<? extends Voxel>> mapper = g -> {
            Controller<? extends Voxel> brain = null;
            Robot<? extends Voxel> robot = null;
            if (controller.equals("phase")) {
                // each element of g becomes a phase
                brain = new TimeFunctions(
                        Grid.create(body.getW(), body.getH(), (x, y) -> (Double t) -> Math.sin(-2 * Math.PI * t + Math.PI * g.get(x + y * body.getW())))
                );
                robot = new Robot<>((Controller<? super ControllableVoxel>) brain, SerializationUtils.clone(body));
            } else if (controller.equals("centralized")) {
                // convert body to sensing body
                Grid<SensingVoxel> sensingBody = Grid.create(body.getW(), body.getH(), (x, y) -> {
                    SensingVoxel sensingVoxel = null;
                    if (body.get(x, y) != null) {
                        sensingVoxel = new SensingVoxel(List.of(
                                new Touch(),
                                new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)),
                                new Normalization(new AreaRatio())
                                ));
                    }
                    return sensingVoxel;
                });
                CentralizedSensing centralizedBrain  = new CentralizedSensing<>(SerializationUtils.clone(sensingBody));
                MultiLayerPerceptron mlp = new MultiLayerPerceptron(
                        MultiLayerPerceptron.ActivationFunction.TANH,
                        centralizedBrain .nOfInputs(),
                        new int[]{(int) (centralizedBrain.nOfInputs() * 0.65d)}, // hidden layers
                        centralizedBrain .nOfOutputs()
                );
                double[] ws = mlp.getParams();
                IntStream.range(0, ws.length).forEach(i -> ws[i] = random.nextGaussian());
                mlp.setParams(ws);
                centralizedBrain.setFunction(mlp);
                robot = new Robot<>(centralizedBrain, SerializationUtils.clone(sensingBody));
            }
            return robot;
        };

        // CMA-ES evolver: https://en.wikipedia.org/wiki/CMA-ES
        Evolver<List<Double>, Robot<? extends Voxel>, Double> evolver = new CMAESEvolver<>(
                mapper,
                new FixedLengthListFactory<>(body.getW() * body.getH(), new UniformDoubleFactory(0, 1)),
                PartialComparator.from(Double.class).reversed().comparing(Individual::getFitness),
                0,
                1
        );

        List<DataCollector<?, ? super Robot<? extends Voxel>, ? super Double>> collectors = List.of(
                new Basic(),
                new Population(),
                new Diversity(),
                new BestInfo("%6.4f"),
                new FunctionOfOneBest<>(i -> List.of(
                        new Item("serialized.robot", it.units.erallab.Utils.safelySerialize(i.getSolution()), "%s")
                ))
        );
        Listener<? super Object, ? super Robot<? extends Voxel>, ? super Double> listener;
        if (statsListenerFactory.getBaseFileName() == null) {
            listener = listener(collectors.toArray(DataCollector[]::new));
        } else {
            listener = statsListenerFactory.build(collectors.toArray(DataCollector[]::new));
        }
        try {
            evolver.solve(
                    Misc.cached(problem.getFitnessFunction(), cacheSize),
                    new Births(births),
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
