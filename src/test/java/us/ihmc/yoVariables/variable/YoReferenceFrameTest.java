package us.ihmc.yoVariables.variable;

import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoReferenceFrameTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000, expected = RuntimeException.class)
   public void testNoEntryKey()
   {
      YoReferenceFrame yoFrame = new YoReferenceFrame("", new YoVariableRegistry(""));
      yoFrame.set(new ReferenceFrame("", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }

         @Override
         public long getNameBasedHashCode()
         {
            return -1L;
         }
      });
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testNullSafety()
   {
      YoReferenceFrame yoFrame = new YoReferenceFrame("", new YoVariableRegistry(""));
      ReferenceFrame someFrame = EuclidFrameRandomTools.nextReferenceFrame(new Random(4293889L));

      assertNull(yoFrame.get());
      yoFrame.set(someFrame);
      assertTrue(yoFrame.get() == someFrame);
      yoFrame.set(null);
      assertNull(yoFrame.get());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testSettingAndGetting()
   {
      YoReferenceFrame yoFrame = new YoReferenceFrame("", new YoVariableRegistry(""));
      Random random = new Random(4293889L);
      ReferenceFrame[] frameTree = EuclidFrameRandomTools.nextReferenceFrameTree(random);

      assertNull(yoFrame.get());
      for (int i = 0; i < 100; i++)
      {
         ReferenceFrame someFrame = frameTree[random.nextInt(frameTree.length)];
         yoFrame.set(someFrame);
         assertTrue(yoFrame.get() == someFrame);
      }
   }
}
