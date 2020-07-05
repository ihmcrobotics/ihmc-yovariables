package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.tuple3D.Point3DBasicsTest;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import org.junit.jupiter.api.BeforeAll;

public class YoPoint3DTest extends Point3DBasicsTest<YoPoint3D>
{
   @BeforeAll
   public static void disableStackTrack()
   {
      YoVariable.SAVE_STACK_TRACE = false;
   }

   @Override
   public YoPoint3D createEmptyTuple()
   {
      YoRegistry registry = new YoRegistry("testYoVariableRegistry");

      return new YoPoint3D("testYoPoint3D", registry);
   }

   @Override
   public YoPoint3D createTuple(double x, double y, double z)
   {
      YoRegistry registry = new YoRegistry("testYoVariableRegistry");

      YoPoint3D point = new YoPoint3D("testYoPoint3D", registry);

      point.set(x, y, z);

      return point;
   }

   @Override
   public YoPoint3D createRandomTuple(Random random)
   {
      YoRegistry registry = new YoRegistry("testYoVariableRegistry");

      YoPoint3D point = new YoPoint3D("testYoPoint3D", registry);

      point.set(random.nextDouble(), random.nextDouble(), random.nextDouble());

      return point;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-12;
   }
}