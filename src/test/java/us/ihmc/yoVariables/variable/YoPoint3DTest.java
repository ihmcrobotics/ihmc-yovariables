package us.ihmc.yoVariables.variable;

import java.util.Random;

import us.ihmc.euclid.tuple3D.Point3DBasicsTest;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoPoint3DTest extends Point3DBasicsTest<YoPoint3D>
{
   @Override
   public YoPoint3D createEmptyTuple()
   {
      YoRegistry registry = new YoRegistry("testYoRegistry");

      return new YoPoint3D("testYoPoint3D", registry);
   }

   @Override
   public YoPoint3D createTuple(double x, double y, double z)
   {
      YoRegistry registry = new YoRegistry("testYoRegistry");

      YoPoint3D point = new YoPoint3D("testYoPoint3D", registry);

      point.set(x, y, z);

      return point;
   }

   @Override
   public YoPoint3D createRandomTuple(Random random)
   {
      YoRegistry registry = new YoRegistry("testYoRegistry");

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