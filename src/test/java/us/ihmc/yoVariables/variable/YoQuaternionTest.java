package us.ihmc.yoVariables.variable;

import java.util.Random;

import us.ihmc.euclid.tuple4D.QuaternionBasicsTest;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoQuaternionTest extends QuaternionBasicsTest<YoQuaternion>
{
   @Override
   public YoQuaternion createEmptyTuple()
   {
      YoRegistry registry = new YoRegistry("testYoRegistry");

      return new YoQuaternion("testYoQuaternion", registry);
   }

   @Override
   public YoQuaternion createTuple(double x, double y, double z, double s)
   {
      YoRegistry registry = new YoRegistry("testYoRegistry");

      YoQuaternion quaternion = new YoQuaternion("testYoQuaternion", registry);

      quaternion.setUnsafe(x, y, z, s);

      return quaternion;
   }

   @Override
   public YoQuaternion createRandomTuple(Random random)
   {
      YoRegistry registry = new YoRegistry("testYoRegistry");

      YoQuaternion quaternion = new YoQuaternion("testYoQuaternion", registry);

      quaternion.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());

      return quaternion;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-12;
   }
}