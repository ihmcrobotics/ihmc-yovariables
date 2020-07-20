package us.ihmc.yoVariables.dataBuffer;

import us.ihmc.yoVariables.variable.YoVariable;

public interface YoBufferVariableEntryHolder
{
   public YoBufferVariableEntryReader getEntry(YoVariable yoVariable);
}
