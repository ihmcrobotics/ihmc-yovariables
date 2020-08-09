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

import java.util.function.IntSupplier;

import us.ihmc.yoVariables.exceptions.IllegalOperationException;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Integer parameter.
 *
 * @author Jesper Smith
 * @see YoParameter
 */
public class IntegerParameter extends YoParameter implements IntSupplier
{
   private static final int DefaultSuggestedMinimum = -10;
   private static final int DefaultSuggestedMaximum = 10;

   /** The variable backing this parameter. */
   private final YoInteger value;
   /** Optional default value used for initializing this parameter. */
   private final int initialValue;

   /**
    * Creates a new integer parameter and registers it to the given registry.
    *
    * @param name     the parameter's name. Must be unique in the registry.
    * @param registry initial parent registry for this parameter.
    */
   public IntegerParameter(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Creates a new integer parameter and registers it to the given registry.
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
   public IntegerParameter(String name, YoRegistry registry, int lowerBound, int upperBound)
   {
      this(name, "", registry, lowerBound, upperBound);
   }

   /**
    * Creates a new integer parameter and registers it to the given registry.
    *
    * @param name        the parameter's name. Must be unique in the registry.
    * @param description description of this parameter's purpose.
    * @param registry    initial parent registry for this parameter.
    */
   public IntegerParameter(String name, String description, YoRegistry registry)
   {
      this(name, description, registry, 0);
   }

   /**
    * Creates a new integer parameter and registers it to the given registry.
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
   public IntegerParameter(String name, String description, YoRegistry registry, int lowerBound, int upperBound)
   {
      this(name, description, registry, 0, lowerBound, upperBound);
   }

   /**
    * Creates a new integer parameter and registers it to the given registry.
    *
    * @param name         the parameter's name. Must be unique in the registry.
    * @param registry     initial parent registry for this parameter.
    * @param initialValue Value to set to when no value can be found in the user provided
    *                     parameterLoader
    */
   public IntegerParameter(String name, YoRegistry registry, int initialValue)
   {
      this(name, "", registry, initialValue);
   }

   /**
    * Creates a new integer parameter and registers it to the given registry.
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
   public IntegerParameter(String name, YoRegistry registry, int initialValue, int lowerBound, int upperBound)
   {
      this(name, "", registry, initialValue, lowerBound, upperBound);
   }

   /**
    * Creates a new integer parameter and registers it to the given registry.
    *
    * @param name         the parameter's name. Must be unique in the registry.
    * @param description  description of this parameter's purpose.
    * @param registry     initial parent registry for this parameter.
    * @param initialValue value to set to when no value can be found in the user provided parameter
    *                     loader.
    */
   public IntegerParameter(String name, String description, YoRegistry registry, int initialValue)
   {
      this(name, description, registry, initialValue, DefaultSuggestedMinimum, DefaultSuggestedMaximum);
   }

   /**
    * Creates a new integer parameter and registers it to the given registry.
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
   public IntegerParameter(String name, String description, YoRegistry registry, int initialValue, int lowerBound, int upperBound)
   {
      value = new YoIntegerParameter(name, description, registry);
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
   public int getAsInt()
   {
      checkLoaded();
      return value.getIntegerValue();
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
   public void setParameterBounds(int lowerBound, int upperBound)
   {
      super.setParameterBounds(lowerBound, upperBound);
   }

   /** {@inheritDoc} */
   @Override
   YoInteger getVariable()
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
    * Internal class to set parameter settings for {@code YoInteger}.
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
      public IntegerParameter getParameter()
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
         newParameter.value.set(value.getAsInt());
         newParameter.loadStatus = getLoadStatus();
         return newParameter.value;
      }
   }

}
