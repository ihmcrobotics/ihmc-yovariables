/*
 * Copyright 2020 Florida Institute for Human and Machine Cognition (IHMC)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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