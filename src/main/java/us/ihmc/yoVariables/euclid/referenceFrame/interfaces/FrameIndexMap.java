package us.ihmc.yoVariables.euclid.referenceFrame.interfaces;

import static gnu.trove.impl.Constants.DEFAULT_CAPACITY;
import static gnu.trove.impl.Constants.DEFAULT_LOAD_FACTOR;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameObject;

/**
 * A map used by the {@link YoMutableFrameObject} to map between frames and frame indices.
 * <p>
 * Several default implementations are provided in this interface.
 *
 * @author Georg Wiedebach
 */
public interface FrameIndexMap
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

   public class FrameIndexHashMap implements FrameIndexMap
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

   public class FrameIndexFinder implements FrameIndexMap
   {
      private final ReferenceFrame rootFrame;

      public FrameIndexFinder(ReferenceFrame frame)
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
}
