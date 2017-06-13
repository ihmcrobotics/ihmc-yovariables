package us.ihmc.yoVariables.listener;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public interface YoVariableRegistryChangedListener
{
   public void yoVariableWasRegistered(YoVariableRegistry registry, YoVariable<?> registeredYoVariable);
   public void yoVariableRegistryWasAdded(YoVariableRegistry addedYoVariableRegistry);
   public void yoVariableRegistryWasCleared(YoVariableRegistry clearedYoVariableRegistry);
}
