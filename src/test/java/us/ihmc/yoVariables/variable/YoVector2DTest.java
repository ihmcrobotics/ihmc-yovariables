package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.tuple2D.Vector2DBasicsTest;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import org.junit.jupiter.api.BeforeAll;

public class YoVector2DTest extends Vector2DBasicsTest<YoVector2D>
{
   @BeforeAll
   public static void disableStackTrack()
   {
      YoVariable.SAVE_STACK_TRACE = false;
   }

   @Override
   public YoVector2D createEmptyTuple()
   {
      YoRegistry registry = new YoRegistry("testYoVariableRegistry");

      return new YoVector2D("testYoVector2D", registry);
   }

   @Override
   public YoVector2D createTuple(double x, double y)
   {
      YoRegistry registry = new YoRegistry("testYoVariableRegistry");

      YoVector2D vector = new YoVector2D("testYoVector2D", registry);

      vector.set(x, y);

      return vector;
   }

   @Override
   public YoVector2D createRandomTuple(Random random)
   {
      YoRegistry registry = new YoRegistry("testYoVariableRegistry");

      YoVector2D vector = new YoVector2D("testYoVector2D", registry);

      vector.set(random.nextDouble(), random.nextDouble());

      return vector;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-12;
   }
}