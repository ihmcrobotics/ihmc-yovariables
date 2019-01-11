package us.ihmc.yoVariables.variable.frameObjects;

import static gnu.trove.impl.Constants.DEFAULT_CAPACITY;
import static gnu.trove.impl.Constants.DEFAULT_LOAD_FACTOR;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.TObjectLongMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import gnu.trove.map.hash.TObjectLongHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;

/**
 * A mapper used by the {@link YoMutableFrameObject} to map between frames and frame indices.
 * <p>
 * Several default implementations are provided in this interface.
 *
 * @author Georg Wiedebach
 *
 */
public interface FrameIndexMapper
{
   static final long NO_ENTRY_KEY = -1;

   public void put(ReferenceFrame referenceFrame);

   public ReferenceFrame getReferenceFrame(long frameIndex);

   public long getFrameIndex(ReferenceFrame referenceFrame);

   public default void putAll(Collection<ReferenceFrame> referenceFrames)
   {
      referenceFrames.forEach(referenceFrame -> put(referenceFrame));
   }

   public default void putAll(List<ReferenceFrame> referenceFrames)
   {
      for (int i = 0; i < referenceFrames.size(); i++)
      {
         put(referenceFrames.get(i));
      }
   }

   public default void putAll(ReferenceFrame[] referenceFrames)
   {
      for (int i = 0; i < referenceFrames.length; i++)
      {
         put(referenceFrames[i]);
      }
   }

   public class DefaultFrameIndexMapper implements FrameIndexMapper
   {
      private final TLongObjectMap<ReferenceFrame> frameIndexMap = new TLongObjectHashMap<>(DEFAULT_CAPACITY, DEFAULT_LOAD_FACTOR, NO_ENTRY_KEY);

      @Override
      public void put(ReferenceFrame referenceFrame)
      {
         if (referenceFrame != null)
         {
            frameIndexMap.putIfAbsent(referenceFrame.getFrameIndex(), referenceFrame);
         }
      }

      @Override
      public ReferenceFrame getReferenceFrame(long frameIndex)
      {
         if (frameIndex == NO_ENTRY_KEY)
         {
            return null;
         }
         ReferenceFrame referenceFrame = frameIndexMap.get(frameIndex);
         if (referenceFrame != null)
         {
            return referenceFrame;
         }
         throw new RuntimeException("Can not get the frame for index " + frameIndex + " because the frame index is unknown to the mapper.");
      }

      @Override
      public long getFrameIndex(ReferenceFrame referenceFrame)
      {
         if (referenceFrame == null)
         {
            return NO_ENTRY_KEY;
         }
         if (frameIndexMap.containsValue(referenceFrame))
         {
            return referenceFrame.getFrameIndex();
         }
         throw new RuntimeException("Can not get the frame index for reference frame " + referenceFrame.getName() + " because frame is unknown to the mapper.");
      }
   }

   public class SearchingFrameIndexMapper implements FrameIndexMapper
   {
      private final ReferenceFrame rootFrame;

      public SearchingFrameIndexMapper(ReferenceFrame frame)
      {
         rootFrame = frame.getRootFrame();
      }

      @Override
      public void put(ReferenceFrame referenceFrame)
      {
         if (referenceFrame.getRootFrame() != rootFrame)
         {
            throw new RuntimeException("Root frame of " + referenceFrame.getName() + " is " + referenceFrame.getRootFrame().getName()
                  + ". This mapper only supports frame in the tree with root frame " + rootFrame.getName() + ".");
         }
      }

      @Override
      public ReferenceFrame getReferenceFrame(long frameIndex)
      {
         if (frameIndex == NO_ENTRY_KEY)
         {
            return null;
         }
         Collection<ReferenceFrame> allFramesInTree = ReferenceFrameTools.getAllFramesInTree(rootFrame);
         Optional<ReferenceFrame> match = allFramesInTree.stream().filter(referenceFrame -> referenceFrame.getFrameIndex() == frameIndex).findFirst();
         if (match.isPresent())
         {
            return match.get();
         }
         throw new RuntimeException("Can not get the frame for index " + frameIndex + " because the frame index is unknown to the mapper.");
      }

      @Override
      public long getFrameIndex(ReferenceFrame referenceFrame)
      {
         if (referenceFrame == null)
         {
            return NO_ENTRY_KEY;
         }
         return referenceFrame.getFrameIndex();
      }
   }

   public class ImmutableFrameIndexMapper implements FrameIndexMapper
   {
      private final TLongObjectMap<ReferenceFrame> frameIndexMap = new TLongObjectHashMap<>();
      private final TObjectLongMap<ReferenceFrame> frameMap = new TObjectLongHashMap<>();

      public ImmutableFrameIndexMapper(TLongObjectMap<ReferenceFrame> frameIndexMap)
      {
         this.frameIndexMap.putAll(frameIndexMap);
         this.frameIndexMap.forEachEntry((frameId, referenceFrame) -> {
            frameMap.put(referenceFrame, frameId);
            return true;
         });
      }

      @Override
      public void put(ReferenceFrame referenceFrame)
      {
         if (!frameMap.containsKey(referenceFrame))
         {
            throw new RuntimeException("This mapper does not support adding new frames through put. Reference frame " + referenceFrame.getName()
                  + " is unknown to the mapper.");
         }
      }

      @Override
      public ReferenceFrame getReferenceFrame(long frameIndex)
      {
         if (!frameIndexMap.containsKey(frameIndex))
         {
            throw new RuntimeException("This mapper does not contain a frame index " + frameIndex + ".");
         }
         return frameIndexMap.get(frameIndex);
      }

      @Override
      public long getFrameIndex(ReferenceFrame referenceFrame)
      {
         if (!frameMap.containsKey(referenceFrame))
         {
            throw new RuntimeException("This mapper does not contain reference frame " + referenceFrame.getName() + ".");
         }
         return frameMap.get(referenceFrame);
      }
   }
}
