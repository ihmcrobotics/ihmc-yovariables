package us.ihmc.yoVariables.listener;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Listener on changes made to a {@link YoVariableRegistry}.
 */
public interface YoVariableRegistryChangedListener
{
   /**
    * Called when registrying a YoVariable with {@link YoVariableRegistry#addYoVariable(YoVariable)}
    *
    * @param registry             YoVariableRegistry the listener is attached to
    * @param registeredYoVariable YoVariable being added to {@code registry}
    */
   public void yoVariableWasRegistered(YoVariableRegistry registry, YoVariable<?> registeredYoVariable);

   /**
    * Called when adding a child registry with {@link YoVariableRegistry#addChild(YoVariableRegistry)}
    * or {@link YoVariableRegistry#addChild(YoVariableRegistry, boolean)} and {@code notifyListeners}
    * is {@code true}
    *
    * @param addedYoVariableRegistry child YoVariableRegistry being added
    */
   public void yoVariableRegistryWasAdded(YoVariableRegistry addedYoVariableRegistry);

   /**
    * Called when clearing a YoVariableRegistry with {@link YoVariableRegistry#clear()}
    *
    * @param clearedYoVariableRegistry registry after being cleared
    */
   public void yoVariableRegistryWasCleared(YoVariableRegistry clearedYoVariableRegistry);
}
