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

import java.util.function.DoubleSupplier;

import us.ihmc.yoVariables.exceptions.IllegalOperationException;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Double parameter.
 *
 * @author Jesper Smith
 * @see YoParameter
 */
public class DoubleParameter extends YoParameter implements DoubleSupplier
{
   private static final double defaultSuggestedMinimum = 0.0;
   private static final double defaultSuggestedMaximum = 1.0;

   /** The variable backing this parameter. */
   private final YoDouble value;
   /** Optional default value used for initializing this parameter. */
   private final double initialValue;

   /**
    * Creates a new double parameter and registers it to the given registry.
    *
    * @param name     the parameter's name. Must be unique in the registry.
    * @param registry initial parent registry for this parameter.
    */
   public DoubleParameter(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Creates a new double parameter and registers it to the given registry.
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
   public DoubleParameter(String name, YoRegistry registry, double lowerBound, double upperBound)
   {
      this(name, "", registry, lowerBound, upperBound);
   }

   /**
    * Creates a new double parameter and registers it to the given registry.
    *
    * @param name        the parameter's name. Must be unique in the registry.
    * @param description description of this parameter's purpose.
    * @param registry    initial parent registry for this parameter.
    */
   public DoubleParameter(String name, String description, YoRegistry registry)
   {
      this(name, description, registry, Double.NaN);
   }

   /**
    * Creates a new double parameter and registers it to the given registry.
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
   public DoubleParameter(String name, String description, YoRegistry registry, double lowerBound, double upperBound)
   {
      this(name, description, registry, Double.NaN, lowerBound, upperBound);
   }

   /**
    * Creates a new double parameter and registers it to the given registry.
    *
    * @param name         the parameter's name. Must be unique in the registry.
    * @param registry     initial parent registry for this parameter.
    * @param initialValue value to set to when no value can be found in the user provided parameter
    *                     loader.
    */
   public DoubleParameter(String name, YoRegistry registry, double initialValue)
   {
      this(name, "", registry, initialValue);
   }

   /**
    * Creates a new double parameter and registers it to the given registry.
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
   public DoubleParameter(String name, YoRegistry registry, double initialValue, double lowerBound, double upperBound)
   {
      this(name, "", registry, initialValue, lowerBound, upperBound);
   }

   /**
    * Creates a new double parameter and registers it to the given registry.
    *
    * @param name         the parameter's name. Must be unique in the registry.
    * @param description  description of this parameter's purpose.
    * @param registry     initial parent registry for this parameter.
    * @param initialValue value to set to when no value can be found in the user provided parameter
    *                     loader.
    */
   public DoubleParameter(String name, String description, YoRegistry registry, double initialValue)
   {
      this(name, description, registry, initialValue, defaultSuggestedMinimum, defaultSuggestedMaximum);
   }

   /**
    * Creates a new double parameter and registers it to the given registry.
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
   public DoubleParameter(String name, String description, YoRegistry registry, double initialValue, double lowerBound, double upperBound)
   {
      value = new YoDoubleParameter(name, description, registry);
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
   public double getAsDouble()
   {
      checkLoaded();
      return value.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public void setParameterBounds(double lowerBound, double upperBound)
   {
      super.setParameterBounds(lowerBound, upperBound);
   }

   /** {@inheritDoc} */
   @Override
   YoDouble getVariable()
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
    * Internal class to set parameter settings for {@code YoDouble}.
    *
    * @author Jesper Smith
    */
   private class YoDoubleParameter extends YoDouble
   {
      public YoDoubleParameter(String name, String description, YoRegistry registry)
      {
         super(name, description, registry);
      }

      @Override
      public boolean isParameter()
      {
         return true;
      }

      @Override
      public DoubleParameter getParameter()
      {
         return DoubleParameter.this;
      }

      @Override
      public YoDouble duplicate(YoRegistry newRegistry)
      {
         DoubleParameter newParameter = new DoubleParameter(getName(), getDescription(), newRegistry, initialValue, getLowerBound(), getUpperBound());
         newParameter.value.set(value.getAsDouble());
         newParameter.loadStatus = getLoadStatus();
         return newParameter.value;
      }
   }
}
