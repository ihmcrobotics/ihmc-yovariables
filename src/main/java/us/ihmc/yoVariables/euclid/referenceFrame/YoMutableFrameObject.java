package us.ihmc.yoVariables.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoLong;

public class YoMutableFrameObject implements ReferenceFrameHolder
{
   private final YoLong frameId;
   private final FrameIndexMap frameIndexMap;

   public YoMutableFrameObject(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      frameId = new YoLong(YoFrameVariableNameTools.createName(namePrefix, "frame", nameSuffix), registry);
      frameIndexMap = new FrameIndexMap.FrameIndexHashMap();
   }

   public YoMutableFrameObject(YoLong frameId, FrameIndexMap frameIndexMap)
   {
      this.frameId = frameId;
      this.frameIndexMap = frameIndexMap;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return frameIndexMap.getReferenceFrame(frameId.getValue());
   }

   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      frameIndexMap.put(referenceFrame);
      frameId.set(frameIndexMap.getFrameIndex(referenceFrame));
   }

   public long getFrameIndex()
   {
      return frameId.getValue();
   }

   public YoLong getYoFrameIndex()
   {
      return frameId;
   }

   public FrameIndexMap getFrameIndexMap()
   {
      return frameIndexMap;
   }
}
