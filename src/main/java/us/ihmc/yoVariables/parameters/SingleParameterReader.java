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
package us.ihmc.yoVariables.parameters;

/**
 * Provides tools for initializing a single parameter.
 * <p>
 * The main usecase for this class is for deserialization of parameters.
 * </p>
 */
public class SingleParameterReader
{
   /**
    * Initializes the given parameter from the given double value.
    * 
    * @param parameter   the parameter to be initialized.
    * @param doubleValue the initial value to use on the parameter.
    * @param loadStatus  indicator of the origin of the given double value.
    * @throws RuntimeException if {@code loadedStatus == ParameterLoadStatus.UNLOADED}.
    */
   public static void readParameter(YoParameter parameter, double doubleValue, ParameterLoadStatus loadStatus)
   {
      if (loadStatus == ParameterLoadStatus.UNLOADED)
      {
         throw new RuntimeException("Can not load parameter and set the status to unloaded.");
      }
      parameter.getVariable().setValueFromDouble(doubleValue);
      parameter.loadStatus = loadStatus;
   }
}
