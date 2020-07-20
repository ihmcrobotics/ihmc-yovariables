package us.ihmc.yoVariables.buffer.interfaces;

import us.ihmc.yoVariables.buffer.YoBuffer;

/**
 * Interface that receives notifications of changes to the current index of a {@link YoBuffer}.
 */
public interface YoBufferIndexChangedListener
{
   /**
    * Called after a change has been made to the current index of a {@link YoBuffer}.
    *
    * @param newIndex the new buffer index.
    */
   void indexChanged(int newIndex);
}
