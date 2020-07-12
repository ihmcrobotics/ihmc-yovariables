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
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Long parameter
 *
 * @author Jesper Smith
 */
public class LongParameter extends YoParameter implements LongProvider
{
   private final static long DefaultSuggestedMinimum = -100L;
   private final static long DefaultSuggestedMaximum = 100L;

   private final YoLong value;
   private final long initialValue;

   /**
    * Create a new Long parameter, registered to the namespace of the registry.
    *
    * @param name     Desired name. Must be unique in the registry
    * @param registry YoRegistry to store under
    */
   public LongParameter(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new Long parameter, registered to the namespace of the registry.
    *
    * @param name             Desired name. Must be unique in the registry
    * @param registry         YoRegistry to store under
    * @param suggestedMinimum A suggested minimum value for this parameter. Not enforced.
    * @param suggestedMaximum A suggested maximum value for this parameter. Not enforced.
    */
   public LongParameter(String name, YoRegistry registry, long suggestedMinimum, long suggestedMaximum)
   {
      this(name, "", registry, suggestedMinimum, suggestedMaximum);
   }

   /**
    * Create a new Long parameter, registered to the namespace of the registry.
    *
    * @param name        Desired name. Must be unique in the registry
    * @param description User readable description that describes the purpose of this parameter
    * @param registry    YoRegistry to store under
    */
   public LongParameter(String name, String description, YoRegistry registry)
   {
      this(name, "", registry, 0L);
   }

   /**
    * Create a new Long parameter, registered to the namespace of the registry.
    *
    * @param name             Desired name. Must be unique in the registry
    * @param description      User readable description that describes the purpose of this parameter
    * @param registry         YoRegistry to store under
    * @param suggestedMinimum A suggested minimum value for this parameter. Not enforced.
    * @param suggestedMaximum A suggested maximum value for this parameter. Not enforced.
    */
   public LongParameter(String name, String description, YoRegistry registry, long suggestedMinimum, long suggestedMaximum)
   {
      this(name, "", registry, 0L, suggestedMinimum, suggestedMaximum);
   }

   /**
    * Create a new Long parameter, registered to the namespace of the registry.
    *
    * @param name         Desired name. Must be unique in the registry
    * @param registry     YoRegistry to store under
    * @param initialValue Value to set to when no value can be found in the user provided
    *                     parameterLoader
    */
   public LongParameter(String name, YoRegistry registry, long initialValue)
   {
      this(name, "", registry, initialValue);
   }

   /**
    * Create a new Long parameter, registered to the namespace of the registry.
    *
    * @param name             Desired name. Must be unique in the registry
    * @param registry         YoRegistry to store under
    * @param initialValue     Value to set to when no value can be found in the user provided
    *                         parameterLoader
    * @param suggestedMinimum A suggested minimum value for this parameter. Not enforced.
    * @param suggestedMaximum A suggested maximum value for this parameter. Not enforced.
    */
   public LongParameter(String name, YoRegistry registry, long initialValue, long suggestedMinimum, long suggestedMaximum)
   {
      this(name, "", registry, initialValue, suggestedMinimum, suggestedMaximum);
   }

   /**
    * Create a new Long parameter, registered to the namespace of the registry.
    *
    * @param name         Desired name. Must be unique in the registry
    * @param description  User readable description that describes the purpose of this parameter
    * @param registry     YoRegistry to store under
    * @param initialValue Value to set to when no value can be found in the user provided
    *                     parameterLoader
    */
   public LongParameter(String name, String description, YoRegistry registry, long initialValue)
   {
      this(name, description, registry, initialValue, DefaultSuggestedMinimum, DefaultSuggestedMaximum);
   }

   /**
    * Create a new Long parameter, registered to the namespace of the registry.
    *
    * @param name             Desired name. Must be unique in the registry
    * @param description      User readable description that describes the purpose of this parameter
    * @param registry         YoRegistry to store under
    * @param initialValue     Value to set to when no value can be found in the user provided
    *                         parameterLoader
    * @param suggestedMinimum A suggested minimum value for this parameter. Not enforced.
    * @param suggestedMaximum A suggested maximum value for this parameter. Not enforced.
    */
   public LongParameter(String name, String description, YoRegistry registry, long initialValue, long suggestedMinimum, long suggestedMaximum)
   {
      super(name, description);

      value = new YoLongParameter(name, description, registry);
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
   public long getValue()
   {
      checkLoaded();
      return value.getLongValue();
   }

   /**
    * Sets the suggested range for tuning purposes. The minimum and maximum will not be enforced and
    * the parameter can be set to any value. This is just a suggestion to the user.
    *
    * @param min Lower end of the suggested range for this parameter.
    * @param max Upper end of the suggested range for this parameter.
    */
   public void setSuggestedRange(long min, long max)
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
      value.set(Long.parseLong(valueString));
   }

   @Override
   void setToDefault()
   {
      value.set(initialValue);
   }

   /**
    * Internal class to set parameter settings for YoLong
    *
    * @author Jesper Smith
    */
   private class YoLongParameter extends YoLong
   {

      public YoLongParameter(String name, String description, YoRegistry registry)
      {
         super(name, description, registry);
      }

      @Override
      public boolean isParameter()
      {
         return true;
      }

      @Override
      public LongParameter getParameter()
      {
         return LongParameter.this;
      }

      @Override
      public YoLong duplicate(YoRegistry newRegistry)
      {
         LongParameter newParameter = new LongParameter(getName(),
                                                        getDescription(),
                                                        newRegistry,
                                                        initialValue,
                                                        (long) getLowerBound(),
                                                        (long) getUpperBound());
         newParameter.value.set(value.getValue());
         newParameter.loadStatus = getLoadStatus();
         return newParameter.value;
      }
   }

}
