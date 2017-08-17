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
package us.ihmc.yoVariables.parameters;

import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Base class for parameters. 
 * 
 * Parameters cannot be changed from code and are only changed by the 
 * user/operator.
 * 
 * Available implementations
 * - BooleanParameter
 * - DoubleParameter
 * - EnumParameter
 * - IntegerParameter
 * - LongParameter
 * 
 * 
 * @author Jesper Smith
 *
 * @param <T>
 */
public abstract class YoParameter<T extends YoParameter<T>>
{
   private final String name;
   private boolean loaded = false;

   public String getName()
   {
      return name;
   }

   YoParameter(String name)
   {
      checkForIllegalCharacters(name);
      this.name = name;
   }

   abstract YoVariable<?> getVariable();

   abstract void setToString(String valueString);

   abstract void setToDefault();

   private static void checkForIllegalCharacters(String name)
   {
      if (YoVariable.ILLEGAL_CHARACTERS.matcher(name).find())
      {
         throw new RuntimeException(name
               + " is an invalid name for a Parameter. A Parameter cannot have crazy characters in them, otherwise namespaces will not work.");
      }
   }

   void checkLoaded()
   {
      if (!loaded)
      {
         throw new RuntimeException("Cannot use parameter " + name + " before it's value is loaded.");
      }
   }

   void load(String valueString)
   {
      if (loaded)
      {
         throw new RuntimeException("Trying to load parameter value twice. Can only load once");
      }

      setToString(valueString);

      loaded = true;
   }

   void loadDefault()
   {
      if (loaded)
      {
         throw new RuntimeException("Trying to load parameter value twice. Can only load once");
      }

      setToDefault();

      loaded = true;
   }

}
