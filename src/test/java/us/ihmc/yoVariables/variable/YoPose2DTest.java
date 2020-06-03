package us.ihmc.yoVariables.variable;

import java.util.Random;

import org.junit.jupiter.api.BeforeAll;

import us.ihmc.euclid.geometry.Pose2DBasicsTest;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoPose2DTest extends Pose2DBasicsTest<YoPose2D>
{
   @BeforeAll
   public static void disableStackTrack()
   {
      YoVariable.SAVE_STACK_TRACE = false;
   }

   @Override
   public YoPose2D createEmptyPose2D()
   {
      return new YoPose2D("blop", new YoVariableRegistry("dummy"));
   }

   @Override
   public YoPose2D createRandomPose2D(Random random)
   {
      YoPose2D pose = createEmptyPose2D();
      pose.set(EuclidGeometryRandomTools.nextPose2D(random));
      return pose;
   }
}