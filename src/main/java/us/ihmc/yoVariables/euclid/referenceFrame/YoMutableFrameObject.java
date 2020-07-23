package us.ihmc.yoVariables.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * Base implementation for managing the reference frame of a frame geometry that is backed by a
 * {@code YoVariable}.
 */
public class YoMutableFrameObject implements ReferenceFrameHolder
{
   private final YoLong frameId;
   private final FrameIndexMap frameIndexMap;

   /**
    * Creates a new {@code YoMutableFrameObject}.
    * 
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoMutableFrameObject(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      frameId = new YoLong(YoFrameVariableNameTools.createName(namePrefix, "frame", nameSuffix), registry);
      frameIndexMap = new FrameIndexMap.FrameIndexHashMap();
   }

   /**
    * Creates a new {@code YoMutableFrameObject} using the given {@code YoVariable} and
    * {@code FrameIndexMap} to use internally.
    * 
    * @param frameId       an existing variable representing the index of the current frame.
    * @param frameIndexMap the frame index manager used to store and retrieve a reference frame.
    */
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

   /**
    * Sets the current reference frame.
    * 
    * @param referenceFrame the new reference frame.
    */
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      frameIndexMap.put(referenceFrame);
      frameId.set(frameIndexMap.getFrameIndex(referenceFrame));
   }

   /**
    * Returns the index of the current reference frame.
    * 
    * @return the frame index.
    */
   public long getFrameIndex()
   {
      return frameId.getValue();
   }

   /**
    * Returns the internal reference to the index of the current reference frame.
    * 
    * @return the frame index as {@code YoLong}.
    */
   public YoLong getYoFrameIndex()
   {
      return frameId;
   }

   /**
    * Returns the internal reference to the frame index map used to store and retrieve reference frames
    * from an index.
    * 
    * @return the index map.
    */
   public FrameIndexMap getFrameIndexMap()
   {
      return frameIndexMap;
   }
}
