package us.ihmc.yoVariables.euclid;

import java.util.Random;

import us.ihmc.euclid.geometry.Pose3DBasicsTest;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoPose3DTest extends Pose3DBasicsTest<YoPose3D>
{
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

   @Override
   public YoPose3D copy(YoPose3D original)
   {
      YoPose3D pose = createEmptyPose3D();
      pose.set(original);
      return pose;
   }

   @Override
   public YoPose3D createRandomTransform2D(Random random)
   {
      YoPose3D pose = createEmptyPose3D();
      pose.getPosition().set(EuclidCoreRandomTools.nextPoint3D(random));
      pose.getOrientation().setToYawOrientation(EuclidCoreRandomTools.nextDouble(random, Math.PI));
      return pose;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-12;
   }

   @Override
   public YoPose3D identity()
   {
      return createEmptyPose3D();
   }
}