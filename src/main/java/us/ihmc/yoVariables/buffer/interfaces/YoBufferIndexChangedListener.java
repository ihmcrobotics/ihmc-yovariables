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

import us.ihmc.yoVariables.buffer.YoBuffer;

/**
 * Interface that receives notifications of changes to the current index of a {@link YoBuffer}.
 */
public interface YoBufferIndexChangedListener
{
   /**
    * Called after a change has been made to the current index of a {@link YoBuffer}.
    *
    * @param newIndex the new buffer index.
    */
   void indexChanged(int newIndex);
}
