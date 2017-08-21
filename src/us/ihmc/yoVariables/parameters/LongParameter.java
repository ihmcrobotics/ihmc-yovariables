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

import us.ihmc.yoVariables.providers.LongProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Long parameter
 * 
 * @author Jesper Smith
 *
 */
public class LongParameter extends YoParameter<LongParameter> implements LongProvider
{
   private final YoLong value;
   private final long initialValue;
   
   /**
    * Create a new Long parameter, registered to the namespace of the registry.
    * 
    * @param name Desired name. Must be unique in the registry
    * @param registry YoVariableRegistry to store under
    */
   public LongParameter(String name, YoVariableRegistry registry)
   {
      this(name, registry, 0L);
   }

   /**
    * Create a new Long parameter, registered to the namespace of the registry.
    * 
    * @param name Desired name. Must be unique in the registry
    * @param registry YoVariableRegistry to store under
    * @param initialValue Value to set to when no value can be found in the user provided parameterLoader
    */
   public LongParameter(String name, YoVariableRegistry registry, long initialValue)
   {
      super(name);

      this.value = new YoLongParameter(name, registry);      
      this.initialValue = initialValue;
   }

   /**
    * Get the current value.
    * 
    * 
    * @return value for this parameter
    * @throws RuntimeException if the parameter is not loaded yet.
    */
   public long getValue()
   {
      checkLoaded();
      return this.value.getLongValue();
   }


   @Override
   public String getValueAsString()
   {
      return String.valueOf(getValue());
   }

   
   @Override
   YoVariable<?> getVariable()
   {
      return this.value;
   }

   @Override
   void setToString(String valueString)
   {
      this.value.set(Long.parseLong(valueString));
   }

   @Override
   void setToDefault()
   {
      this.value.set(initialValue);
   }
   
   /**
    * Internal class to set parameter settings for YoLong 
    * 
    * @author Jesper Smith
    *
    */
   private class YoLongParameter extends YoLong
   {

      public YoLongParameter(String name, YoVariableRegistry registry)
      {
         super(name, registry);
      }
      
      @Override
      public boolean isParameter()
      {
         return true;
      }
      
      @Override
      public YoParameter<?> getParameter()
      {
         return LongParameter.this;
      }
   }

   
}
