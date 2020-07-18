package us.ihmc.yoVariables.variable.frameObjects;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.Assertions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameObject;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoMutableFrameObjectTest
{
   @Test
   public void testMutableFrameObject()
   {
      YoRegistry registry = new YoRegistry("TestRegistry");
      Random random = new Random(4290L);

      List<ReferenceFrame> frames = new ArrayList<>();
      frames.addAll(Arrays.asList(EuclidFrameRandomTools.nextReferenceFrameTree(random)));
      frames.add(null);

      YoMutableFrameObject mutableFrameObject = new YoMutableFrameObject("", "", registry);
      YoVariable frameIndex = registry.findVariable(YoFrameVariableNameTools.createName("", "frame", ""));
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
   public void testFrameIndexHashMap()
   {
      Random random = new Random(4290L);

      List<ReferenceFrame> frames = new ArrayList<>();
      frames.addAll(Arrays.asList(EuclidFrameRandomTools.nextReferenceFrameTree(random)));
      frames.add(null);

      FrameIndexMap mapper = new FrameIndexMap.FrameIndexHashMap();
      mapper.putAll(frames);

      frames.forEach(frame ->
      {
         if (frame == null)
         {
            assertEquals(FrameIndexMap.NO_ENTRY_KEY, mapper.getFrameIndex(frame));
            assertTrue(mapper.getReferenceFrame(FrameIndexMap.NO_ENTRY_KEY) == frame);
         }
         else
         {
            assertEquals(frame.getFrameIndex(), mapper.getFrameIndex(frame));
            assertTrue(mapper.getReferenceFrame(frame.getFrameIndex()) == frame);
         }
      });

      // Test unknown frame fails:
      ReferenceFrame unknown = ReferenceFrameTools.constructFrameWithUnchangingTranslationFromParent("Unknown", ReferenceFrame.getWorldFrame(), new Vector3D());
      Assertions.assertExceptionThrown(RuntimeException.class, () -> mapper.getFrameIndex(unknown));
      Assertions.assertExceptionThrown(RuntimeException.class, () -> mapper.getReferenceFrame(unknown.getFrameIndex()));
   }

   @Test
   public void testFrameIndexFinder()
   {
      // Create map but do not put any frames in!
      FrameIndexMap mapper = new FrameIndexMap.FrameIndexFinder(ReferenceFrame.getWorldFrame());

      Random random = new Random(4290L);
      List<ReferenceFrame> frames = new ArrayList<>();
      frames.addAll(Arrays.asList(EuclidFrameRandomTools.nextReferenceFrameTree(random)));
      frames.add(null);

      frames.forEach(frame ->
      {
         if (frame == null)
         {
            assertEquals(FrameIndexMap.NO_ENTRY_KEY, mapper.getFrameIndex(frame));
            assertTrue(mapper.getReferenceFrame(FrameIndexMap.NO_ENTRY_KEY) == frame);
         }
         else
         {
            assertEquals(frame.getFrameIndex(), mapper.getFrameIndex(frame));
            assertTrue(mapper.getReferenceFrame(frame.getFrameIndex()) == frame);
         }
      });

      long frameIndex = frames.get(5).getFrameIndex();
      assertTrue(frameIndex > 0);

      // This is a problem with this map implementation. Frames can get collected.
      mapper.getReferenceFrame(frameIndex);
      frames = null;
      System.gc();
      Assertions.assertExceptionThrown(RuntimeException.class, () -> mapper.getReferenceFrame(frameIndex));
   }
}
