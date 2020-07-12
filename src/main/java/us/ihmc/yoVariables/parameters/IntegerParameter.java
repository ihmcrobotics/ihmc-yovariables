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

import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Integer parameter
 *
 * @author Jesper Smith
 */
public class IntegerParameter extends YoParameter<IntegerParameter> implements IntegerProvider
{
   private static final int DefaultSuggestedMinimum = -10;
   private static final int DefaultSuggestedMaximum = 10;

   private final YoInteger value;
   private final int initialValue;

   /**
    * Create a new Integer parameter, registered to the namespace of the registry.
    *
    * @param name     Desired name. Must be unique in the registry
    * @param registry YoRegistry to store under
    */
   public IntegerParameter(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new Integer parameter, registered to the namespace of the registry.
    *
    * @param name             Desired name. Must be unique in the registry
    * @param registry         YoRegistry to store under
    * @param suggestedMinimum A suggested minimum value for this parameter. Not enforced.
    * @param suggestedMaximum A suggested maximum value for this parameter. Not enforced.
    */
   public IntegerParameter(String name, YoRegistry registry, int suggestedMinimum, int suggestedMaximum)
   {
      this(name, "", registry, suggestedMinimum, suggestedMaximum);
   }

   /**
    * Create a new Integer parameter, registered to the namespace of the registry.
    *
    * @param name        Desired name. Must be unique in the registry
    * @param description User readable description that describes the purpose of this parameter
    * @param registry    YoRegistry to store under
    */
   public IntegerParameter(String name, String description, YoRegistry registry)
   {
      this(name, "", registry, 0);
   }

   /**
    * Create a new Integer parameter, registered to the namespace of the registry.
    *
    * @param name             Desired name. Must be unique in the registry
    * @param description      User readable description that describes the purpose of this parameter
    * @param registry         YoRegistry to store under
    * @param suggestedMinimum A suggested minimum value for this parameter. Not enforced.
    * @param suggestedMaximum A suggested maximum value for this parameter. Not enforced.
    */
   public IntegerParameter(String name, String description, YoRegistry registry, int suggestedMinimum, int suggestedMaximum)
   {
      this(name, "", registry, 0, suggestedMinimum, suggestedMaximum);
   }

   /**
    * Create a new Integer parameter, registered to the namespace of the registry.
    *
    * @param name         Desired name. Must be unique in the registry
    * @param registry     YoRegistry to store under
    * @param initialValue Value to set to when no value can be found in the user provided
    *                     parameterLoader
    */
   public IntegerParameter(String name, YoRegistry registry, int initialValue)
   {
      this(name, "", registry, initialValue);
   }

   /**
    * Create a new Integer parameter, registered to the namespace of the registry.
    *
    * @param name             Desired name. Must be unique in the registry
    * @param registry         YoRegistry to store under
    * @param initialValue     Value to set to when no value can be found in the user provided
    *                         parameterLoader
    * @param suggestedMinimum A suggested minimum value for this parameter. Not enforced.
    * @param suggestedMaximum A suggested maximum value for this parameter. Not enforced.
    */
   public IntegerParameter(String name, YoRegistry registry, int initialValue, int suggestedMinimum, int suggestedMaximum)
   {
      this(name, "", registry, initialValue, suggestedMinimum, suggestedMaximum);
   }

   /**
    * Create a new Integer parameter, registered to the namespace of the registry.
    *
    * @param name         Desired name. Must be unique in the registry
    * @param description  User readable description that describes the purpose of this parameter
    * @param registry     YoRegistry to store under
    * @param initialValue Value to set to when no value can be found in the user provided
    *                     parameterLoader
    */
   public IntegerParameter(String name, String description, YoRegistry registry, int initialValue)
   {
      this(name, description, registry, initialValue, DefaultSuggestedMinimum, DefaultSuggestedMaximum);
   }

   /**
    * Create a new Integer parameter, registered to the namespace of the registry.
    *
    * @param name             Desired name. Must be unique in the registry
    * @param description      User readable description that describes the purpose of this parameter
    * @param registry         YoRegistry to store under
    * @param initialValue     Value to set to when no value can be found in the user provided
    *                         parameterLoader
    * @param suggestedMinimum A suggested minimum value for this parameter. Not enforced.
    * @param suggestedMaximum A suggested maximum value for this parameter. Not enforced.
    */
   public IntegerParameter(String name, String description, YoRegistry registry, int initialValue, int suggestedMinimum, int suggestedMaximum)
   {
      super(name, description);

      value = new YoIntegerParameter(name, description, registry);
      this.initialValue = initialValue;

      setSuggestedRange(suggestedMinimum, suggestedMaximum);
   }

   /**
    * Get the current value.
    *
    * @return value for this parameter
    * @throws RuntimeException if the parameter is not loaded yet.
    */
   @Override
   public int getValue()
   {
      checkLoaded();
      return value.getIntegerValue();
   }

   /**
    * Sets the suggested range for tuning purposes. The minimum and maximum will not be enforced and
    * the parameter can be set to any value. This is just a suggestion to the user.
    *
    * @param min Lower end of the suggested range for this parameter.
    * @param max Upper end of the suggested range for this parameter.
    */
   public void setSuggestedRange(int min, int max)
   {
      super.setSuggestedRange(min, max);
   }

   @Override
   public String getValueAsString()
   {
      return String.valueOf(getValue());
   }

   @Override
   YoVariable getVariable()
   {
      return value;
   }

   @Override
   void setToString(String valueString)
   {
      value.set(Integer.parseInt(valueString));
   }

   @Override
   void setToDefault()
   {
      value.set(initialValue);
   }

   /**
    * Internal class to set parameter settings for YoInteger
    *
    * @author Jesper Smith
    */
   private class YoIntegerParameter extends YoInteger
   {

      public YoIntegerParameter(String name, String description, YoRegistry registry)
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
         return IntegerParameter.this;
      }

      @Override
      public YoInteger duplicate(YoRegistry newRegistry)
      {
         IntegerParameter newParameter = new IntegerParameter(getName(),
                                                              getDescription(),
                                                              newRegistry,
                                                              initialValue,
                                                              (int) getLowerBound(),
                                                              (int) getUpperBound());
         newParameter.value.set(value.getValue());
         newParameter.loadStatus = getLoadStatus();
         return newParameter.value;
      }
   }

}
