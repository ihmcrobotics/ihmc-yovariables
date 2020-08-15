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

/**
 * Minimalist interface for a class managing a time variable buffer.
 */
public interface YoTimeBufferHolder
{
   /**
    * Gets a copy of the buffer for the time variable.
    * 
    * @return the copy of the buffer for the time variable.
    */
   double[] getTimeBuffer();
}
