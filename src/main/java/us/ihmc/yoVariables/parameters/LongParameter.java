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
import us.ihmc.yoVariables.providers.LongProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * Long parameter.
 *
 * @author Jesper Smith
 */
public class LongParameter extends YoParameter implements LongProvider
{
   private final static long DefaultSuggestedMinimum = -100L;
   private final static long DefaultSuggestedMaximum = 100L;

   /** The variable backing this parameter. */
   private final YoLong value;
   /** Optional default value used for initializing this parameter. */
   private final long initialValue;

   /**
    * Creates a new long parameter and registers it to the given registry.
    *
    * @param name     Desired name. Must be unique in the registry
    * @param registry YoRegistry to store under
    */
   public LongParameter(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Creates a new long parameter and registers it to the given registry.
    * <p>
    * Parameter bounds are typically used when interacting with the parameter via a GUI. For instance,
    * the parameter bounds can be used to set the bounds of control slider or set the range when
    * plotting this variable.
    * </p>
    * <p>
    * Note that nothing in the implementation of a {@code YoParameter} enforces the value to remain
    * within its current bounds, it is only for facilitating the definition of bounds and tracking to
    * the bounds' owner.
    * </p>
    *
    * @param name       the parameter's name. Must be unique in the registry.
    * @param registry   initial parent registry for this parameter.
    * @param lowerBound double value representing the lower bound for this parameter. Not enforced.
    * @param upperBound double value representing the upper bound for this parameter. Not enforced.
    */
   public LongParameter(String name, YoRegistry registry, long lowerBound, long upperBound)
   {
      this(name, "", registry, lowerBound, upperBound);
   }

   /**
    * Creates a new long parameter and registers it to the given registry.
    *
    * @param name        the parameter's name. Must be unique in the registry.
    * @param description description of this parameter's purpose.
    * @param registry    initial parent registry for this parameter.
    */
   public LongParameter(String name, String description, YoRegistry registry)
   {
      this(name, "", registry, 0L);
   }

   /**
    * Creates a new long parameter and registers it to the given registry.
    * <p>
    * Parameter bounds are typically used when interacting with the parameter via a GUI. For instance,
    * the parameter bounds can be used to set the bounds of control slider or set the range when
    * plotting this variable.
    * </p>
    * <p>
    * Note that nothing in the implementation of a {@code YoParameter} enforces the value to remain
    * within its current bounds, it is only for facilitating the definition of bounds and tracking to
    * the bounds' owner.
    * </p>
    *
    * @param name        the parameter's name. Must be unique in the registry.
    * @param description description of this parameter's purpose.
    * @param registry    initial parent registry for this parameter.
    * @param lowerBound  double value representing the lower bound for this parameter. Not enforced.
    * @param upperBound  double value representing the upper bound for this parameter. Not enforced.
    */
   public LongParameter(String name, String description, YoRegistry registry, long lowerBound, long upperBound)
   {
      this(name, "", registry, 0L, lowerBound, upperBound);
   }

   /**
    * Creates a new long parameter and registers it to the given registry.
    *
    * @param name         the parameter's name. Must be unique in the registry.
    * @param registry     initial parent registry for this parameter.
    * @param initialValue value to set to when no value can be found in the user provided parameter
    *                     loader.
    */
   public LongParameter(String name, YoRegistry registry, long initialValue)
   {
      this(name, "", registry, initialValue);
   }

   /**
    * Creates a new long parameter and registers it to the given registry.
    * <p>
    * Parameter bounds are typically used when interacting with the parameter via a GUI. For instance,
    * the parameter bounds can be used to set the bounds of control slider or set the range when
    * plotting this variable.
    * </p>
    * <p>
    * Note that nothing in the implementation of a {@code YoParameter} enforces the value to remain
    * within its current bounds, it is only for facilitating the definition of bounds and tracking to
    * the bounds' owner.
    * </p>
    *
    * @param name         the parameter's name. Must be unique in the registry.
    * @param registry     initial parent registry for this parameter.
    * @param initialValue value to set to when no value can be found in the user provided parameter
    *                     loader.
    * @param lowerBound   double value representing the lower bound for this parameter. Not enforced.
    * @param upperBound   double value representing the upper bound for this parameter. Not enforced.
    */
   public LongParameter(String name, YoRegistry registry, long initialValue, long lowerBound, long upperBound)
   {
      this(name, "", registry, initialValue, lowerBound, upperBound);
   }

   /**
    * Creates a new long parameter and registers it to the given registry.
    *
    * @param name         the parameter's name. Must be unique in the registry.
    * @param description  description of this parameter's purpose.
    * @param registry     initial parent registry for this parameter.
    * @param initialValue value to set to when no value can be found in the user provided parameter
    *                     loader.
    */
   public LongParameter(String name, String description, YoRegistry registry, long initialValue)
   {
      this(name, description, registry, initialValue, DefaultSuggestedMinimum, DefaultSuggestedMaximum);
   }

   /**
    * Creates a new long parameter and registers it to the given registry.
    * <p>
    * Parameter bounds are typically used when interacting with the parameter via a GUI. For instance,
    * the parameter bounds can be used to set the bounds of control slider or set the range when
    * plotting this variable.
    * </p>
    * <p>
    * Note that nothing in the implementation of a {@code YoParameter} enforces the value to remain
    * within its current bounds, it is only for facilitating the definition of bounds and tracking to
    * the bounds' owner.
    * </p>
    *
    * @param name         the parameter's name. Must be unique in the registry.
    * @param description  description of this parameter's purpose.
    * @param registry     initial parent registry for this parameter.
    * @param initialValue value to set to when no value can be found in the user provided parameter
    *                     loader.
    * @param lowerBound   double value representing the lower bound for this parameter. Not enforced.
    * @param upperBound   double value representing the upper bound for this parameter. Not enforced.
    */
   public LongParameter(String name, String description, YoRegistry registry, long initialValue, long lowerBound, long upperBound)
   {
      value = new YoLongParameter(name, description, registry);
      this.initialValue = initialValue;

      setParameterBounds(lowerBound, upperBound);
   }

   /**
    * Get the current value.
    *
    * @return value for this parameter.
    * @throws IllegalOperationException if the parameter is not loaded yet.
    */
   @Override
   public long getValue()
   {
      checkLoaded();
      return value.getLongValue();
   }

   /**
    * Sets the bounds for this parameter's range of values.
    * <p>
    * Parameter bounds are typically used when interacting with the parameter via a GUI. For instance,
    * the parameter bounds can be used to set the bounds of control slider or set the range when
    * plotting this variable.
    * </p>
    * <p>
    * Note that nothing in the implementation of a {@code YoParameter} enforces the value to remain
    * within its current bounds, it is only for facilitating the definition of bounds and tracking to
    * the bounds' owner.
    * </p>
    *
    * @param lowerBound double value representing the lower bound for this parameter. Not enforced.
    * @param upperBound double value representing the upper bound for this parameter. Not enforced.
    */
   public void setParameterBounds(long lowerBound, long upperBound)
   {
      super.setParameterBounds(lowerBound, upperBound);
   }

   /** {@inheritDoc} */
   @Override
   YoLong getVariable()
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
    * Internal class to set parameter settings for {@code YoLong}.
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
         LongParameter newParameter = new LongParameter(getName(), getDescription(), newRegistry, initialValue, (long) getLowerBound(), (long) getUpperBound());
         newParameter.value.set(value.getValue());
         newParameter.loadStatus = getLoadStatus();
         return newParameter.value;
      }
   }

}
