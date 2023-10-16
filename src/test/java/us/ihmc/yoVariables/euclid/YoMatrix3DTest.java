package us.ihmc.yoVariables.euclid;

import us.ihmc.euclid.matrix.Matrix3DBasicsTest;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

public class YoMatrix3DTest extends Matrix3DBasicsTest<YoMatrix3D>
{
   @Override
   public YoMatrix3D createEmptyMatrix()
   {
      YoRegistry registry = new YoRegistry("testYoRegistry");

      return new YoMatrix3D("testYoMatrix3D", registry);
   }

   @Override
   public YoMatrix3D createMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      YoRegistry registry = new YoRegistry("testYoRegistry");

      YoMatrix3D matrix = new YoMatrix3D("testYoMatrix3D", registry);
      matrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      return matrix;
   }

   @Override
   public YoMatrix3D createRandomMatrix(Random random)
   {
      YoRegistry registry = new YoRegistry("testYoRegistry");

      YoMatrix3D matrix = new YoMatrix3D("testYoMatrix3D", registry);

      matrix.set(random.nextDouble(),
                 random.nextDouble(),
                 random.nextDouble(),
                 random.nextDouble(),
                 random.nextDouble(),
                 random.nextDouble(),
                 random.nextDouble(),
                 random.nextDouble(),
                 random.nextDouble());
      return matrix;
   }
}
