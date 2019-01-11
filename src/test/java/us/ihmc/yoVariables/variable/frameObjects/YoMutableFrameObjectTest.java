package us.ihmc.yoVariables.variable.frameObjects;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.TObjectLongMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import gnu.trove.map.hash.TObjectLongHashMap;
import us.ihmc.commons.Assertions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.Assert;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoMutableFrameObjectTest
{
   @Test
   public void testMutableFrameObject()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      Random random = new Random(4290L);

      List<ReferenceFrame> frames = new ArrayList<>();
      frames.addAll(Arrays.asList(EuclidFrameRandomTools.nextReferenceFrameTree(random)));
      frames.add(null);

      YoMutableFrameObject mutableFrameObject = new YoMutableFrameObject("", "", registry);
      YoVariable<?> frameIndex = registry.getVariable(YoFrameVariableNameTools.createName("", "frame", ""));
      Assert.assertNotNull(frameIndex);

      for (int i = 0; i < 1000; i++)
      {
         ReferenceFrame frame = frames.get(random.nextInt(frames.size()));
         mutableFrameObject.setReferenceFrame(frame);
         Assert.assertTrue(frame == mutableFrameObject.getReferenceFrame());
         if (frame == null)
         {
            Assert.assertEquals(FrameIndexMapper.NO_ENTRY_KEY, frameIndex.getValueAsLongBits());
         }
         else
         {
            Assert.assertEquals(frame.getFrameIndex(), frameIndex.getValueAsLongBits());
         }
      }
   }

   @Test
   public void testDefaultMapper()
   {
      Random random = new Random(4290L);

      List<ReferenceFrame> frames = new ArrayList<>();
      frames.addAll(Arrays.asList(EuclidFrameRandomTools.nextReferenceFrameTree(random)));
      frames.add(null);

      FrameIndexMapper mapper = new FrameIndexMapper.DefaultFrameIndexMapper();
      mapper.putAll(frames);

      frames.forEach(frame -> {
         if (frame == null)
         {
            Assert.assertEquals(FrameIndexMapper.NO_ENTRY_KEY, mapper.getFrameIndex(frame));
            Assert.assertTrue(mapper.getReferenceFrame(FrameIndexMapper.NO_ENTRY_KEY) == frame);
         }
         else
         {
            Assert.assertEquals(frame.getFrameIndex(), mapper.getFrameIndex(frame));
            Assert.assertTrue(mapper.getReferenceFrame(frame.getFrameIndex()) == frame);
         }
      });

      // Test unknown frame fails:
      ReferenceFrame unknown = ReferenceFrameTools.constructFrameWithUnchangingTranslationFromParent("Unknown", ReferenceFrame.getWorldFrame(), new Vector3D());
      Assertions.assertExceptionThrown(RuntimeException.class, () -> mapper.getFrameIndex(unknown));
      Assertions.assertExceptionThrown(RuntimeException.class, () -> mapper.getReferenceFrame(unknown.getFrameIndex()));
   }

   @Test
   public void testSearchingMapper()
   {
      // Create map but do not put any frames in!
      FrameIndexMapper mapper = new FrameIndexMapper.SearchingFrameIndexMapper(ReferenceFrame.getWorldFrame());

      Random random = new Random(4290L);
      List<ReferenceFrame> frames = new ArrayList<>();
      frames.addAll(Arrays.asList(EuclidFrameRandomTools.nextReferenceFrameTree(random)));
      frames.add(null);

      frames.forEach(frame -> {
         if (frame == null)
         {
            Assert.assertEquals(FrameIndexMapper.NO_ENTRY_KEY, mapper.getFrameIndex(frame));
            Assert.assertTrue(mapper.getReferenceFrame(FrameIndexMapper.NO_ENTRY_KEY) == frame);
         }
         else
         {
            Assert.assertEquals(frame.getFrameIndex(), mapper.getFrameIndex(frame));
            Assert.assertTrue(mapper.getReferenceFrame(frame.getFrameIndex()) == frame);
         }
      });

      long frameIndex = frames.get(5).getFrameIndex();
      Assert.assertTrue(frameIndex > 0);

      // This is a problem with this map implementation. Frames can get collected.
      mapper.getReferenceFrame(frameIndex);
      frames = null;
      System.gc();
      Assertions.assertExceptionThrown(RuntimeException.class, () -> mapper.getReferenceFrame(frameIndex));
   }

   @Test
   public void testImmutableMapper()
   {
      Random random = new Random(4290L);

      List<ReferenceFrame> frames = new ArrayList<>();
      frames.addAll(Arrays.asList(EuclidFrameRandomTools.nextReferenceFrameTree(random)));
      frames.add(null);

      TLongObjectMap<ReferenceFrame> frameIndexMap = new TLongObjectHashMap<>();
      TObjectLongMap<ReferenceFrame> indexFrameMap = new TObjectLongHashMap<>();
      frames.forEach(frame -> {
         long randomIndex = random.nextLong();
         if (frameIndexMap.put(randomIndex, frame) != null)
         {
            Assert.fail("Really unlikely random long collision. Change your seed.");
         }
         indexFrameMap.put(frame, randomIndex);
      });

      FrameIndexMapper mapper = new FrameIndexMapper.ImmutableFrameIndexMapper(frameIndexMap);

      // Putting existing frames is fine and does not change anything.
      mapper.putAll(frames);

      frames.forEach(frame -> {
         Assert.assertEquals(indexFrameMap.get(frame), mapper.getFrameIndex(frame));
         Assert.assertTrue(mapper.getReferenceFrame(indexFrameMap.get(frame)) == frame);
      });

      // Change original map and make sure nothing changed.
      frameIndexMap.clear();
      frames.forEach(frame -> {
         long randomIndex = random.nextLong();
         if (frameIndexMap.put(randomIndex, frame) != null)
         {
            Assert.fail("Really unlikely random long collision. Change your seed.");
         }
      });

      frames.forEach(frame -> {
         Assert.assertEquals(indexFrameMap.get(frame), mapper.getFrameIndex(frame));
         Assert.assertTrue(mapper.getReferenceFrame(indexFrameMap.get(frame)) == frame);
      });

      // Test unknown frame fails:
      ReferenceFrame unknown = ReferenceFrameTools.constructFrameWithUnchangingTranslationFromParent("Unknown", ReferenceFrame.getWorldFrame(), new Vector3D());
      Assertions.assertExceptionThrown(RuntimeException.class, () -> mapper.put(unknown));
      Assertions.assertExceptionThrown(RuntimeException.class, () -> mapper.getFrameIndex(unknown));
      Assertions.assertExceptionThrown(RuntimeException.class, () -> mapper.getReferenceFrame(unknown.getFrameIndex()));
   }
}
