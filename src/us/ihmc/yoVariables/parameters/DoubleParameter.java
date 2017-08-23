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

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Double parameter
 * 
 * @author Jesper Smith
 *
 */
public class DoubleParameter extends YoParameter<DoubleParameter> implements DoubleProvider
{
   private final YoDouble value;
   private final double initialValue;
   
   /**
    * Create a new Double parameter, registered to the namespace of the registry.
    * 
    * @param name Desired name. Must be unique in the registry
    * @param registry YoVariableRegistry to store under
    */
   public DoubleParameter(String name, YoVariableRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new Double parameter, registered to the namespace of the registry.
    * 
    * @param name Desired name. Must be unique in the registry
    * @param description User readable description that describes the purpose of this parameter 
    * @param registry YoVariableRegistry to store under
    */
   public DoubleParameter(String name, String description, YoVariableRegistry registry)
   {
      this(name, "", registry, Double.NaN);
   }

   /**
    * Create a new Double parameter, registered to the namespace of the registry.
    * 
    * @param name Desired name. Must be unique in the registry
    * @param registry YoVariableRegistry to store under
    * @param initialValue Value to set to when no value can be found in the user provided parameterLoader
    */
   public DoubleParameter(String name, YoVariableRegistry registry, double initialValue)
   {
      this(name, "", registry, initialValue);
   }

    /**
    * Create a new Double parameter, registered to the namespace of the registry.
    * 
    * @param name Desired name. Must be unique in the registry
    * @param description User readable description that describes the purpose of this parameter
    * @param registry YoVariableRegistry to store under
    * @param initialValue Value to set to when no value can be found in the user provided parameterLoader
    */
   public DoubleParameter(String name, String description, YoVariableRegistry registry, double initialValue)
   {
      super(name, description);

      this.value = new YoDoubleParameter(name, description, registry);      
      this.initialValue = initialValue;
   }

   /**
    * Get the current value.
    * 
    * 
    * @return value for this parameter
    * @throws RuntimeException if the parameter is not loaded yet.
    */
   public double getValue()
   {
      checkLoaded();
      return this.value.getDoubleValue();
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
      this.value.set(Double.parseDouble(valueString));
   }

   @Override
   void setToDefault()
   {
      this.value.set(initialValue);
   }
   
   /**
    * Internal class to set parameter settings for YoDouble 
    * 
    * @author Jesper Smith
    *
    */
   private class YoDoubleParameter extends YoDouble
   {

      public YoDoubleParameter(String name, String description, YoVariableRegistry registry)
      {
         super(name, description, registry);
      }
      
      @Override
      public boolean isParameter()
      {
         return true;
      }
      
      @Override
      public YoParameter<?> getParameter()
      {
         return DoubleParameter.this;
      }
   }

   
}
