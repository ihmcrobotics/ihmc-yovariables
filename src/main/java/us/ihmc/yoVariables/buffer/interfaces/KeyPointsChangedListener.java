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
package us.ihmc.yoVariables.buffer.interfaces;

import java.util.List;

/**
 * Interface that receives notifications of changes to a {@link KeyPointsHolder}.
 */
public interface KeyPointsChangedListener
{
   /**
    * Called after a change has been made to a {@link KeyPointsHolder}.
    * 
    * @param change an object representing the change that was done.
    * @see Change
    */
   void changed(Change change);

   /**
    * Represents a report of a single change done to a {@link KeyPointsHolder}.
    */
   public static interface Change
   {
      /**
       * Indicates that the enable flag was toggled.
       * 
       * @return {@code true} if the enable flag was changed, {@code false} otherwise.
       * @see #areKeyPointsEnabled()
       */
      boolean wasToggled();

      /**
       * Returns the current state of the enable flag.
       * 
       * @return the enable flag current value.
       */
      boolean areKeyPointsEnabled();

      /**
       * Returns the index list of key points that were added.
       * 
       * @return the new key points or an empty list if no key points were added.
       */
      List<Integer> getAddedKeyPoints();

      /**
       * Returns the index list of key points that were removed.
       * 
       * @return the key points that were removed or an empty list if no key points were removed.
       */
      List<Integer> getRemovedKeyPoints();
   }
}
