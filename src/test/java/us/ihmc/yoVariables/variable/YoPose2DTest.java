package us.ihmc.yoVariables.variable;

import java.util.Random;

import us.ihmc.euclid.geometry.Pose2DBasicsTest;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoPose2DTest extends Pose2DBasicsTest<YoPose2D>
{
   @Override
   public YoPose2D createEmptyPose2D()
   {
      return new YoPose2D("blop", new YoRegistry("dummy"));
   }

   @Override
   public YoPose2D createRandomPose2D(Random random)
   {
      YoPose2D pose = createEmptyPose2D();
      pose.set(EuclidGeometryRandomTools.nextPose2D(random));
      return pose;
   }
}