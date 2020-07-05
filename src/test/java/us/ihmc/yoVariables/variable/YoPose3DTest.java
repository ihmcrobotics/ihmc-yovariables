package us.ihmc.yoVariables.variable;

import java.util.Random;

import org.junit.jupiter.api.BeforeAll;

import us.ihmc.euclid.geometry.Pose3DBasicsTest;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoPose3DTest extends Pose3DBasicsTest<YoPose3D>
{
   @BeforeAll
   public static void disableStackTrack()
   {
      YoVariable.SAVE_STACK_TRACE = false;
   }

   @Override
   public YoPose3D createEmptyPose3D()
   {
      return new YoPose3D("blop", new YoRegistry("dummy"));
   }

   @Override
   public YoPose3D createRandomPose3D(Random random)
   {
      YoPose3D pose = createEmptyPose3D();
      pose.set(EuclidGeometryRandomTools.nextPose3D(random));
      return pose;
   }
}