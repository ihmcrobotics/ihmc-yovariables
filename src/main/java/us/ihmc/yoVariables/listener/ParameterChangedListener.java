/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC)
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
package us.ihmc.yoVariables.listener;

import us.ihmc.yoVariables.parameters.YoParameter;

/**
 * Listener for changing parameters
 *
 * @author Jesper Smith
 */
public interface ParameterChangedListener
{
   /**
    * Called when the primitive data-type backing a YoParameter is changed
    *
    * @param v YoParameter the listener is attached to
    */
   public void notifyOfParameterChange(YoParameter<?> v);

}
