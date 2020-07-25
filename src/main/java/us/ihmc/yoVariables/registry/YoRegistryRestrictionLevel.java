package us.ihmc.yoVariables.registry;

import us.ihmc.yoVariables.variable.YoVariable;

/**
 * <p>
 * The constant are declared in order of restriction, i.e. when comparing two levels of restriction,
 * the one with greatest ordinal is the most restrictive.
 * </p>
 */
public enum YoRegistryRestrictionLevel
{
   /**
    * When fully mutable, the structure of this registry and all its descendants can be modified
    * without any restriction, i.e. the user can add and remove {@link YoRegistry} and
    * {@link YoVariable}.
    */
   FULLY_MUTABLE(true, true),
   /**
    * When restricted, the structure of this registry add all its descendants can only be expanded,
    * i.e. the user can add {@link YoRegistry} and {@link YoVariable} but can not remove them.
    */
   RESTRICTED(true, false),
   /**
    * When immutable, the structure of this registry and all its descendants cannot be modified, i.e.
    * the user cannot add nor remove {@link YoRegistry} or {@link YoVariable}.
    */
   IMMUTABLE(false, false);

   private final boolean additionAllowed;
   private final boolean removalAllowed;

   YoRegistryRestrictionLevel(boolean additionAllowed, boolean removalAllowed)
   {
      this.additionAllowed = additionAllowed;
      this.removalAllowed = removalAllowed;
   }

   /**
    * Returns whether addition of registry and/or variable is authorized under this restriction.
    * 
    * @return whether addition is allowed.
    */
   public boolean isAdditionAllowed()
   {
      return additionAllowed;
   }

   /**
    * Returns whether removal of registry and/or variable is authorized under this restriction.
    * 
    * @return whether removal is allowed.
    */
   public boolean isRemovalAllowed()
   {
      return removalAllowed;
   }
}