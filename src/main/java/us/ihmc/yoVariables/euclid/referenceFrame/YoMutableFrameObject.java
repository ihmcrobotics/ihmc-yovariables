package us.ihmc.yoVariables.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * Base implementation for managing the reference frame of a frame geometry that is backed by a
 * {@code YoVariable}.
 */
public interface YoMutableFrameObject extends ReferenceFrameHolder
{
   @Override
   default ReferenceFrame getReferenceFrame()
   {
      return getFrameIndexMap().getReferenceFrame(getFrameIndex());
   }

   /**
    * Sets the current reference frame.
    * 
    * @param referenceFrame the new reference frame.
    */
   default void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      getFrameIndexMap().put(referenceFrame);
      getYoFrameIndex().set(getFrameIndexMap().getFrameIndex(referenceFrame));
   }

   /**
    * Returns the index of the current reference frame.
    * 
    * @return the frame index.
    */
   default long getFrameIndex()
   {
      return getYoFrameIndex().getValue();
   }

   /**
    * Returns the internal reference to the index of the current reference frame.
    * 
    * @return the frame index as {@code YoLong}.
    */
   YoLong getYoFrameIndex();

   /**
    * Returns the internal reference to the frame index map used to store and retrieve reference frames
    * from an index.
    * 
    * @return the index map.
    */
   FrameIndexMap getFrameIndexMap();
}
