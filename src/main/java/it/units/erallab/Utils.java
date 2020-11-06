package it.units.erallab;

import java.io.*;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.zip.GZIPInputStream;
import java.util.zip.GZIPOutputStream;

/**
 * @author eric
 * @created 2020/08/19
 * @project VSREvolution
 */
public class Utils {

  private static final Logger L = Logger.getLogger(Utils.class.getName());

  private Utils() {
  }

  public static String safelySerialize(Serializable object) {
    try (
            ByteArrayOutputStream baos = new ByteArrayOutputStream();
            ObjectOutputStream oos = new ObjectOutputStream(new GZIPOutputStream(baos, true))
    ) {
      oos.writeObject(object);
      oos.flush();
      oos.close();
      return Base64.getEncoder().encodeToString(baos.toByteArray());
    } catch (IOException e) {
      L.log(Level.SEVERE, String.format("Cannot serialize due to %s", e), e);
      return "";
    }
  }

  public static <T> T safelyDeserialize(String string, Class<T> tClass) {
    try (
            ByteArrayInputStream bais = new ByteArrayInputStream(Base64.getDecoder().decode(string));
            ObjectInputStream ois = new ObjectInputStream(new GZIPInputStream(bais))
    ) {
      Object o = ois.readObject();
      return (T) o;
    } catch (IOException | ClassNotFoundException e) {
      L.log(Level.SEVERE, String.format("Cannot deserialize due to %s", e), e);
      return null;
    }
  }

  public static double[][] createHillyTerrain(double h, double w, int seed) {
      double TERRAIN_BORDER_HEIGHT = 100d;
      int TERRAIN_LENGHT = 2000;
      Random random = new Random(seed);
      List<Double> xs = new ArrayList<>(List.of(0d, 10d));
      List<Double> ys = new ArrayList<>(List.of(TERRAIN_BORDER_HEIGHT, 0d));
      while (xs.get(xs.size() - 1) < TERRAIN_LENGHT) {
          xs.add(xs.get(xs.size() - 1) + Math.max(1d, (random.nextGaussian() * 0.25 + 1) * w));
          ys.add(ys.get(ys.size() - 1) + random.nextGaussian() * h);
      }
      xs.addAll(List.of(xs.get(xs.size() - 1) + 10, xs.get(xs.size() - 1) + 20));
      ys.addAll(List.of(0d, TERRAIN_BORDER_HEIGHT));
      return new double[][]{
          xs.stream().mapToDouble(d -> d).toArray(),
          ys.stream().mapToDouble(d -> d).toArray()
      };
  }

  private static String paramValue(String pattern, String string, String paramName) {
    Matcher matcher = Pattern.compile(pattern).matcher(string);
    if (matcher.matches()) {
      return matcher.group(paramName);
    }
    throw new IllegalStateException(String.format("Param %s not found in %s with pattern %s", paramName, string, pattern));
  }
}

