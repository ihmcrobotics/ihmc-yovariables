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
package us.ihmc.yoVariables.listener;

import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Listener on the backing value of a {@link YoVariable}.
 *
 * @author Jerry Pratt
 */
public interface YoVariableChangedListener
{
   /**
    * Called when the primitive data-type backing a YoVariable is changed
    *
    * @param source YoVariable the listener is attached to
    */
   void changed(YoVariable source);
}
