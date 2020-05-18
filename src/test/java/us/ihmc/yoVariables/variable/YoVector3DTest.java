package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.tuple3D.Vector3DBasicsTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Random;

import org.junit.jupiter.api.BeforeAll;

public class YoVector3DTest extends Vector3DBasicsTest<YoVector3D>
{
   @BeforeAll
   public static void disableStackTrack()
   {
      YoVariable.SAVE_STACK_TRACE = false;
   }

   @Override
   public YoVector3D createEmptyTuple()
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      return new YoVector3D("testYoVector3D", registry);
   }

   @Override
   public YoVector3D createTuple(double x, double y, double z)
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      YoVector3D vector = new YoVector3D("testYoVector3D", registry);

      vector.set(x, y, z);

      return vector;
   }

   @Override
   public YoVector3D createRandomTuple(Random random)
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

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