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

import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Boolean parameter
 *
 * @author Jesper Smith
 */
public class BooleanParameter extends YoParameter<BooleanParameter> implements BooleanProvider
{
   private final YoBoolean value;
   private final boolean initialValue;

   /**
    * Create a new Boolean parameter, registered to the namespace of the registry.
    *
    * @param name     Desired name. Must be unique in the registry
    * @param registry YoVariableRegistry to store under
    */
   public BooleanParameter(String name, YoVariableRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new Boolean parameter, registered to the namespace of the registry.
    *
    * @param name        Desired name. Must be unique in the registry
    * @param description User readable description that describes the purpose of this parameter
    * @param registry    YoVariableRegistry to store under
    */
   public BooleanParameter(String name, String description, YoVariableRegistry registry)
   {
      this(name, "", registry, false);
   }

   /**
    * Create a new Boolean parameter, registered to the namespace of the registry.
    *
    * @param name         Desired name. Must be unique in the registry
    * @param registry     YoVariableRegistry to store under
    * @param initialValue Value to set to when no value can be found in the user provided
    *                     parameterLoader
    */
   public BooleanParameter(String name, YoVariableRegistry registry, boolean initialValue)
   {
      this(name, "", registry, initialValue);
   }

   /**
    * Create a new Boolean parameter, registered to the namespace of the registry.
    *
    * @param name         Desired name. Must be unique in the registry
    * @param description  User readable description that describes the purpose of this parameter
    * @param registry     YoVariableRegistry to store under
    * @param initialValue Value to set to when no value can be found in the user provided
    *                     parameterLoader
    */
   public BooleanParameter(String name, String description, YoVariableRegistry registry, boolean initialValue)
   {
      super(name, description);

      value = new YoBooleanParameter(name, description, registry);
      this.initialValue = initialValue;

      setSuggestedRange(0, 1);
   }

   /**
    * Get the current value.
    *
    * @return value for this parameter
    * @throws RuntimeException if the parameter is not loaded yet.
    */
   @Override
   public boolean getValue()
   {
      checkLoaded();
      return value.getBooleanValue();
   }

   @Override
   public String getValueAsString()
   {
      return String.valueOf(getValue());
   }

   @Override
   YoVariable<?> getVariable()
   {
      return value;
   }

   @Override
   void setToString(String valueString)
   {
      value.set(Boolean.parseBoolean(valueString));
   }

   @Override
   void setToDefault()
   {
      value.set(initialValue);
   }

   /**
    * Internal class to set parameter settings for YoBoolean
    *
    * @author Jesper Smith
    */
   private class YoBooleanParameter extends YoBoolean
   {

      public YoBooleanParameter(String name, String description, YoVariableRegistry registry)
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
         return BooleanParameter.this;
      }

      @Override
      public YoBoolean duplicate(YoVariableRegistry newRegistry)
      {
         BooleanParameter newParameter = new BooleanParameter(getName(), getDescription(), newRegistry, initialValue);
         newParameter.value.set(value.getValue());
         newParameter.loadStatus = getLoadStatus();
         return newParameter.value;
      }
   }

}
