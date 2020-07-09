package us.ihmc.yoVariables.listener;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Interface that receives notifications of changes to a {@link YoRegistry}.
 */
public interface YoRegistryChangedListener
{
   /**
    * Called after a change has been made to a {@link YoRegistry} or one of its descendants.
    *
    * @param change an object representing the change that was done.
    * @see Change
    */
   void onChanged(Change change);

   /**
    * Represents a report of a single change done to a {@link YoRegistry}.
    */
   public static interface Change
   {
      /**
       * Indicates that a registry was added:
       * <ul>
       * <li>The registry that triggered the event can be accessed via {@link #getSource()}.
       * <li>The registry that was added can be accessed via {@link #getTargetRegistry()}.
       * <li>The parent of the new registry can be accessed via {@link #getTargetParentRegistry()}.
       * <li>{@link #getTargetVariable()} returns {@code null}.
       * </ul>
       * 
       * @return {@code true} if the change was the addition of a new registry.
       */
      boolean wasRegistryAdded();

      /**
       * Indicates that a registry was removed:
       * <ul>
       * <li>The registry that triggered the event can be accessed via {@link #getSource()}.
       * <li>The registry that was removed can be accessed via {@link #getTargetRegistry()}.
       * <li>The parent of the registry before it was removed can be accessed via
       * {@link #getTargetParentRegistry()}.
       * <li>{@link #getTargetVariable()} returns {@code null}.
       * </ul>
       * 
       * @return {@code true} if the change was the removal of a registry.
       */
      boolean wasRegistryRemoved();

      /**
       * Indicates that a variable was added:
       * <ul>
       * <li>The registry that triggered the event can be accessed via {@link #getSource()}.
       * <li>The variable that was added can be accessed via {@link #getTargetVariable()}.
       * <li>The parent registry of the new variable can be accessed via
       * {@link #getTargetParentRegistry()}.
       * <li>{@link #getTargetRegistry()} returns {@code null}.
       * </ul>
       * 
       * @return {@code true} if the change was the addition of a new variable.
       */
      boolean wasVariableAdded();

      /**
       * Indicates that a variable was removed:
       * <ul>
       * <li>The registry that triggered the event can be accessed via {@link #getSource()}.
       * <li>The variable that was removed can be accessed via {@link #getTargetVariable()}.
       * <li>The parent registry of the variable before it was removed can be accessed via
       * {@link #getTargetParentRegistry()}.
       * <li>{@link #getTargetRegistry()} returns {@code null}.
       * </ul>
       * 
       * @return {@code true} if the change was the removal of a variable.
       */
      boolean wasVariableRemoved();

      /**
       * Indicates that a registry was cleared:
       * <ul>
       * <li>The registry that triggered the event is the same as the registry that was cleared and can be
       * accessed via {@link #getSource()}.
       * <li>{@link #getTargetParentRegistry()}, {@link #getTargetRegistry()},
       * {@link #getTargetVariable()} all return {@code null}.
       * </ul>
       * 
       * @return {@code true} if the change was the source of this event being cleared.
       */
      boolean wasCleared();

      /**
       * The registry that fired the change.
       * 
       * @return the source of this change.
       */
      YoRegistry getSource();

      /**
       * The parent registry a variable or registry was added to or removed from or {@code null} when the
       * change is about a registry being cleared.
       * 
       * @return the registry that was directly modified or {@code null} if not applicable.
       */
      YoRegistry getTargetParentRegistry();

      /**
       * The registry that was added or removed or {@code null} when the change is not about the addition
       * or removal of a registry.
       * 
       * @return the added/removed registry, or {@code null} if not applicable.
       */
      YoRegistry getTargetRegistry();

      /**
       * The variable that was added or removed or {@code null} when the change is not about the addition
       * or removal of a variable.
       * 
       * @return the added/removed variable, or {@code null} if not applicable.
       */
      YoVariable<?> getTargetVariable();
   }
}
