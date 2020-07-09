package us.ihmc.yoVariables.variable;

import java.util.Random;

import us.ihmc.euclid.tuple3D.Vector3DBasicsTest;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoVector3DTest extends Vector3DBasicsTest<YoVector3D>
{
   @Override
   public YoVector3D createEmptyTuple()
   {
      YoRegistry registry = new YoRegistry("testYoVariableRegistry");

      return new YoVector3D("testYoVector3D", registry);
   }

   @Override
   public YoVector3D createTuple(double x, double y, double z)
   {
      YoRegistry registry = new YoRegistry("testYoVariableRegistry");

      YoVector3D vector = new YoVector3D("testYoVector3D", registry);

      vector.set(x, y, z);

      return vector;
   }

   @Override
   public YoVector3D createRandomTuple(Random random)
   {
      YoRegistry registry = new YoRegistry("testYoVariableRegistry");

      YoVector3D vector = new YoVector3D("testYoVector3D", registry);

      vector.set(random.nextDouble(), random.nextDouble(), random.nextDouble());

      return vector;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-12;
   }
}