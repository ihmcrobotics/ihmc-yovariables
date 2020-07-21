package us.ihmc.yoVariables.buffer.interfaces;

import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Minimalist interface for a class managing variable buffers.
 */
public interface YoBufferVariableEntryHolder
{
   /**
    * Returns the buffer for the given {@code variable}.
    * 
    * @param variable the variable to find the buffer of.
    * @return the variable's buffer entry or {@code null} if it could not be found.
    */
   public YoBufferVariableEntryReader getEntry(YoVariable variable);
}
