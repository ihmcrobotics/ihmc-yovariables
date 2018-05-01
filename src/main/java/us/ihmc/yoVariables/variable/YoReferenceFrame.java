package us.ihmc.yoVariables.variable;

import gnu.trove.impl.Constants;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoReferenceFrame
{
   /** Represents null in the frame map since the reference frame ID can not be negative. */
   private static final long NO_ENTRY_KEY = -1L;
   private final TLongObjectMap<ReferenceFrame> frameMap = new TLongObjectHashMap<>(Constants.DEFAULT_CAPACITY, Constants.DEFAULT_LOAD_FACTOR, NO_ENTRY_KEY);

   private final YoLong frameId;

   public YoReferenceFrame(String frameName, YoVariableRegistry registry)
   {
      frameId = new YoLong(frameName, registry);
      frameId.set(NO_ENTRY_KEY);
   }

   public void set(ReferenceFrame frame)
   {
      if (frame == null)
      {
         this.frameId.set(NO_ENTRY_KEY);
         return;
      }

      // TODO: change this to the frame ID once euclid 0.8.3 is released.
      long frameId = frame.getNameBasedHashCode();
      if (frameId == NO_ENTRY_KEY)
      {
         throw new RuntimeException("The ID of a reference frame can never be " + NO_ENTRY_KEY);
      }

      if (!frameMap.containsKey(frameId))
      {
         frameMap.put(frameId, frame);
      }

      this.frameId.set(frameId);
   }

   public ReferenceFrame get()
   {
      if (frameId.getValue() == NO_ENTRY_KEY)
      {
         return null;
      }
      return frameMap.get(frameId.getValue());
   }
}
