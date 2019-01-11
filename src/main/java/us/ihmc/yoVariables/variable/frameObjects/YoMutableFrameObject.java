package us.ihmc.yoVariables.variable.frameObjects;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoLong;

public class YoMutableFrameObject
{
   private final YoLong frameId;
   private final FrameIndexMapper frameIndexMapper;

   public YoMutableFrameObject(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      frameId = new YoLong(YoFrameVariableNameTools.createName(namePrefix, "frame", nameSuffix), registry);
      frameIndexMapper = new FrameIndexMapper.DefaultFrameIndexMapper();
      setReferenceFrame(ReferenceFrame.getWorldFrame());
   }

   public YoMutableFrameObject(YoLong frameId, FrameIndexMapper frameIndexMapper)
   {
      this.frameId = frameId;
      this.frameIndexMapper = frameIndexMapper;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return frameIndexMapper.getReferenceFrame(frameId.getValue());
   }

   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      frameIndexMapper.put(referenceFrame);
      frameId.set(frameIndexMapper.getFrameIndex(referenceFrame));
   }
}
