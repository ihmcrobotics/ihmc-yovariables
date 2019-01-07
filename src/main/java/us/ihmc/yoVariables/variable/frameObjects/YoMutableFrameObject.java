package us.ihmc.yoVariables.variable.frameObjects;

import gnu.trove.impl.Constants;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoLong;

public abstract class YoMutableFrameObject
{
   // TODO: extract this to reference frame as an impossible frame ID.
   public static final long NO_ENTRY_KEY = -1;

   private final YoLong frameId;

   private final TLongObjectMap<ReferenceFrame> frameMap = new TLongObjectHashMap<>(Constants.DEFAULT_CAPACITY, Constants.DEFAULT_LOAD_FACTOR,
                                                                                    YoMutableFrameObject.NO_ENTRY_KEY);

   public YoMutableFrameObject(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      frameId = new YoLong(YoFrameVariableNameTools.createName(namePrefix, "frame", nameSuffix), registry);
      setReferenceFrame(ReferenceFrame.getWorldFrame());
   }

   public ReferenceFrame getReferenceFrame()
   {
      return frameMap.get(frameId.getValue());
   }

   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      if (referenceFrame == null)
      {
         frameId.set(frameMap.getNoEntryKey());
      }
      else
      {
         long frameIndex = referenceFrame.getFrameIndex();
         frameMap.putIfAbsent(frameIndex, referenceFrame);
         frameId.set(frameIndex);
      }
   }
}
