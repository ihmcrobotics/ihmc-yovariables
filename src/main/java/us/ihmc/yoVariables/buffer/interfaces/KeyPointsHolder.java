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

import us.ihmc.yoVariables.buffer.KeyPointsHandler;

/**
 * Base interface for a class that manages a collection of buffer key points.
 * <p>
 * A key point can be used to highlight a specific index in the buffer for any reason. The typical
 * usecase is for allowing the user to highlight particular events in the buffer and visualize them
 * via a graphical user interface.
 * </p>
 * 
 * @see KeyPointsHandler
 */
public interface KeyPointsHolder
{
   /**
    * Toggles the "enable" flag associated to this set of key points.
    */
   void toggleKeyPoints();

   /**
    * Returns the current state of the "enable" flag associated to this set of key points.
    * 
    * @return the "enable" flag value.
    */
   boolean areKeyPointsEnabled();

   /**
    * Adds a listener to this key points holder.
    *
    * @param listener the listener for listening to changes done on the key points.
    */
   void addListener(KeyPointsChangedListener listener);
}
