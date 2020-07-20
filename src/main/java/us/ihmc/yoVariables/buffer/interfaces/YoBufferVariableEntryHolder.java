package us.ihmc.yoVariables.buffer.interfaces;

import us.ihmc.yoVariables.variable.YoVariable;

public interface YoBufferVariableEntryHolder
{
   public YoBufferVariableEntryReader getEntry(YoVariable yoVariable);
}
