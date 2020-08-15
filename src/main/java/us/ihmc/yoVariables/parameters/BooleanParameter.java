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

import us.ihmc.yoVariables.exceptions.IllegalOperationException;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * Boolean parameter.
 *
 * @author Jesper Smith
 * @see YoParameter
 */
public class BooleanParameter extends YoParameter implements BooleanProvider
{
   /** The variable backing this parameter. */
   private final YoBoolean value;
   /** Optional default value used for initializing this parameter. */
   private final boolean initialValue;

   /**
    * Creates a new boolean parameter and registers it to the given registry.
    *
    * @param name     the parameter's name. Must be unique in the registry.
    * @param registry initial parent registry for this parameter.
    */
   public BooleanParameter(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Creates a new boolean parameter and registers it to the given registry.
    *
    * @param name        the parameter's name. Must be unique in the registry.
    * @param description description of this parameter's purpose.
    * @param registry    initial parent registry for this parameter.
    */
   public BooleanParameter(String name, String description, YoRegistry registry)
   {
      this(name, description, registry, false);
   }

   /**
    * Creates a new boolean parameter and registers it to the given registry.
    *
    * @param name         the parameter's name. Must be unique in the registry.
    * @param registry     initial parent registry for this parameter.
    * @param initialValue value to set to when no value can be found in the user provided parameter
    *                     loader.
    */
   public BooleanParameter(String name, YoRegistry registry, boolean initialValue)
   {
      this(name, "", registry, initialValue);
   }

   /**
    * Creates a new boolean parameter and registers it to the given registry.
    *
    * @param name         the parameter's name. Must be unique in the registry.
    * @param description  description of this parameter's purpose.
    * @param registry     initial parent registry for this parameter.
    * @param initialValue value to set to when no value can be found in the user provided parameter
    *                     loader.
    */
   public BooleanParameter(String name, String description, YoRegistry registry, boolean initialValue)
   {
      value = new YoBooleanParameter(name, description, registry);
      this.initialValue = initialValue;

      setParameterBounds(0, 1);
   }

   /**
    * Gets the current value.
    *
    * @return value for this parameter
    * @throws IllegalOperationException if the parameter is not loaded yet.
    */
   @Override
   public boolean getValue()
   {
      checkLoaded();
      return value.getBooleanValue();
   }

   /** {@inheritDoc} */
   @Override
   YoBoolean getVariable()
   {
      return value;
   }

   /** {@inheritDoc} */
   @Override
   void setToDefault()
   {
      value.set(initialValue);
   }

   /**
    * Internal class to set parameter settings for {@code YoBoolean}.
    *
    * @author Jesper Smith
    */
   private class YoBooleanParameter extends YoBoolean
   {

      public YoBooleanParameter(String name, String description, YoRegistry registry)
      {
         super(name, description, registry);
      }

      @Override
      public boolean isParameter()
      {
         return true;
      }

      @Override
      public BooleanParameter getParameter()
      {
         return BooleanParameter.this;
      }

      @Override
      public YoBoolean duplicate(YoRegistry newRegistry)
      {
         BooleanParameter newParameter = new BooleanParameter(getName(), getDescription(), newRegistry, initialValue);
         newParameter.value.set(value.getValue());
         newParameter.loadStatus = getLoadStatus();
         return newParameter.value;
      }
   }

}
