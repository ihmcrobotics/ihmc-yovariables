package us.ihmc.yoVariables.listener;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Listener on changes made to a {@link YoRegistry}.
 */
public interface YoVariableRegistryChangedListener
{
   /**
    * Called when registrying a YoVariable with {@link YoRegistry#addYoVariable(YoVariable)}
    *
    * @param registry             YoVariableRegistry the listener is attached to
    * @param registeredYoVariable YoVariable being added to {@code registry}
    */
   public void yoVariableWasRegistered(YoRegistry registry, YoVariable<?> registeredYoVariable);

   /**
    * Called when adding a child registry with {@link YoRegistry#addChild(YoRegistry)}
    * or {@link YoRegistry#addChild(YoRegistry, boolean)} and {@code notifyListeners}
    * is {@code true}
    *
    * @param addedYoVariableRegistry child YoVariableRegistry being added
    */
   public void yoVariableRegistryWasAdded(YoRegistry addedYoVariableRegistry);

   /**
    * Called when clearing a YoVariableRegistry with {@link YoRegistry#clear()}
    *
    * @param clearedYoVariableRegistry registry after being cleared
    */
   public void yoVariableRegistryWasCleared(YoRegistry clearedYoVariableRegistry);
}
