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

import us.ihmc.yoVariables.providers.EnumProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Enum parameter
 * 
 * @author Jesper Smith
 *
 */
public class EnumParameter<T extends Enum<T>> extends YoParameter<EnumParameter<T>> implements EnumProvider<T>
{
   private final YoEnum<T> value;
   private final T initialValue;
   
   /**
    * Create a new Enum parameter, registered to the namespace of the registry.
    * 
    * @param name Desired name. Must be unique in the registry
    * @param registry YoVariableRegistry to store under
    */
   public EnumParameter(String name, YoVariableRegistry registry, Class<T> enumType, boolean allowNullValues)
   {
      this(name, registry, enumType, allowNullValues, allowNullValues ? null : enumType.getEnumConstants()[0]);
   }

   /**
    * Create a new Enum parameter, registered to the namespace of the registry.
    * 
    * @param name Desired name. Must be unique in the registry
    * @param registry YoVariableRegistry to store under
    * @param initialValue Value to set to when no value can be found in the user provided parameterLoader
    */
   public EnumParameter(String name, YoVariableRegistry registry, Class<T> enumType, boolean allowNullValues, T initialValue)
   {
      super(name);

      this.value = new YoEnumParameter(name, registry, enumType, allowNullValues);
      if(!this.value.getAllowNullValue() && initialValue == null)
      {
         throw new RuntimeException("Cannot initialize to null value, allowNullValues is false");
      }
      
      this.initialValue = initialValue;
   }

   /**
    * Get the current value.
    * 
    * 
    * @return value for this parameter
    * @throws RuntimeException if the parameter is not loaded yet.
    */
   public T getValue()
   {
      checkLoaded();
      return this.value.getEnumValue();
   }


   @Override
   public String getValueAsString()
   {
      return this.value.getStringValue();
   }

   
   @Override
   YoVariable<?> getVariable()
   {
      return this.value;
   }

   @Override
   void setToString(String valueString)
   {
      if("null".equalsIgnoreCase(valueString))
      {
         this.value.set(null);
         return;
      }
      
      for(int i = 0; i < this.value.getEnumValues().length; i++)
      {
         if(this.value.getEnumValues()[i].toString().equals(valueString))
         {
            this.value.set(this.value.getEnumValues()[i]);
            return;
         }
      }
      
      throw new RuntimeException("Cannot set enum value to " + valueString + ", undefined enum constant"); 
   }

   @Override
   void setToDefault()
   {
      this.value.set(initialValue);
   }
   
   /**
    * Internal class to set parameter settings for YoEnum 
    * 
    * @author Jesper Smith
    *
    */
   private class YoEnumParameter extends YoEnum<T>
   {

      public YoEnumParameter(String name, YoVariableRegistry registry, Class<T> enumType, boolean allowNullValues)
      {
         super(name, registry, enumType, allowNullValues);
      }
      
      @Override
      public boolean isParameter()
      {
         return true;
      }
      
      @Override
      public YoParameter<?> getParameter()
      {
         return EnumParameter.this;
      }
   }

   
}
