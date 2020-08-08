package us.ihmc.yoVariables.variable.frameObjects;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.YoMutableFrameObject;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoVariable;

class YoMutableFramePose3DTest
{

   @Test
   public void testMutableFrameObject()
   {
      YoRegistry registry = new YoRegistry("TestRegistry");
      Random random = new Random(4290L);

      List<ReferenceFrame> frames = new ArrayList<>();
      frames.addAll(Arrays.asList(EuclidFrameRandomTools.nextReferenceFrameTree(random)));
      frames.add(null);

      YoMutableFrameObject mutableFrameObject = new YoMutableFramePose3D("", "", registry);
      YoVariable frameIndex = registry.findVariable(YoGeometryNameTools.assembleName("", "frame", ""));
      assertNotNull(frameIndex);

      for (int i = 0; i < 1000; i++)
      {
         ReferenceFrame frame = frames.get(random.nextInt(frames.size()));
         mutableFrameObject.setReferenceFrame(frame);
         assertTrue(frame == mutableFrameObject.getReferenceFrame());
         if (frame == null)
         {
            assertEquals(FrameIndexMap.NO_ENTRY_KEY, frameIndex.getValueAsLongBits());
         }
         else
         {
            assertEquals(frame.getFrameIndex(), frameIndex.getValueAsLongBits());
         }
      }
   }

   @Test
   void testConstructors()
   {
      YoRegistry registry = new YoRegistry("dummy");
      new YoMutableFramePose3D("the", "pose", registry);
   }

}
