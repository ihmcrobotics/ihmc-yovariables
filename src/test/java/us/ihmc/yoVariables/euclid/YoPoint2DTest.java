package us.ihmc.yoVariables.euclid;

import java.util.Random;

import us.ihmc.euclid.tuple2D.Point2DBasicsTest;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoPoint2DTest extends Point2DBasicsTest<YoPoint2D>
{
   @Override
   public YoPoint2D createEmptyTuple()
   {
      YoRegistry registry = new YoRegistry("testYoRegistry");

      return new YoPoint2D("testYoPoint2D", registry);
   }

   @Override
   public YoPoint2D createTuple(double x, double y)
   {
      YoRegistry registry = new YoRegistry("testYoRegistry");

      YoPoint2D point = new YoPoint2D("testYoPoint2D", registry);

      point.set(x, y);

      return point;
   }

   @Override
   public YoPoint2D createRandomTuple(Random random)
   {
      YoRegistry registry = new YoRegistry("testYoRegistry");

      YoPoint2D point = new YoPoint2D("testYoPoint2D", registry);

      point.set(random.nextDouble(), random.nextDouble());

      return point;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-12;
   }
}