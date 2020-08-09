/*
 * Copyright 2020 Florida Institute for Human and Machine Cognition (IHMC)
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
package us.ihmc.yoVariables.variable;

import java.util.function.DoubleSupplier;

import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Double implementation of a {@code YoVariable}.
 * 
 * @see YoVariable
 */
public class YoDouble extends YoVariable implements DoubleSupplier
{
   private double value;

   /**
    * Create a new {@code YoDouble} and initializes to {@code 0.0}.
    *
    * @param name     the name for this variable that can be used to retrieve it from a
    *                 {@link YoRegistry}.
    * @param registry initial parent registry for this variable.
    */
   public YoDouble(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new {@code YoDouble} and initializes to {@code 0.0}.
    *
    * @param name        the name for this variable that can be used to retrieve it from a
    *                    {@link YoRegistry}.
    * @param description description of this variable's purpose.
    * @param registry    initial parent registry for this variable.
    */
   public YoDouble(String name, String description, YoRegistry registry)
   {
      super(YoVariableType.DOUBLE, name, description, registry);
      this.set(0.0);
   }

   /**
    * Assesses if this variable's current value is {@link Double#NaN}.
    *
    * @return boolean return of Double.isNaN on this YoDouble's internal double state
    */
   public boolean isNaN()
   {
      return Double.isNaN(value);
   }

   /**
    * Sets this YoDouble to its current value plus the current value of the given YoDouble.
    *
    * @param other YoDouble whose value should be added to this YoDouble
    */
   public void add(YoDouble other)
   {
      this.set(this.value + other.value);
   }

   /**
    * Sets this YoDouble to its current value minus the current value of the given YoDouble.
    *
    * @param other YoDouble whose value should be subtracted from this YoDouble
    */
   public void sub(YoDouble other)
   {
      this.set(this.value - other.value);
   }

   /**
    * Sets this YoDouble to its current value minus the given value.
    *
    * @param value double to subtract from this YoDouble
    */
   public void sub(double value)
   {
      this.set(this.value - value);
   }

   /**
    * Sets this YoDouble to its current value plus the given value.
    *
    * @param value double to add to this YoDouble
    */
   public void add(double value)
   {
      this.set(this.value + value);
   }

   /**
    * Sets this YoDouble to its current value multiplied by the given value.
    *
    * @param value double to multiply this YoDouble by
    */
   public void mul(double value)
   {
      this.set(this.value * value);
   }

   /**
    * Sets this YoDouble to its current value multiplied by the current value of the given YoDouble.
    *
    * @param other YoDouble whose value should be used to multiply this YoDouble's value by
    */
   public void mul(YoDouble other)
   {
      this.set(this.value * other.value);
   }

   /**
    * Check if the value contained by this YoDouble is equal to the given double.
    *
    * @param value double for this YoDouble to be compared to
    * @return boolean if this YoDouble's value is the same as the passed value
    */
   public boolean valueEquals(double value)
   {
      return this.value == value;
   }

   /**
    * Sets this YoDouble's internal double value to {@link Double#NaN}.
    */
   public void setToNaN()
   {
      this.set(Double.NaN);
   }

   /**
    * Retrieves the current double value of this variable.
    *
    * @return the internal double value of this variable.
    */
   @Override
   public double getAsDouble()
   {
      return value;
   }

   /**
    * Retrieves the current double value of this variable.
    *
    * @return the internal double value of this variable.
    */
   public double getDoubleValue()
   {
      return value;
   }

   /**
    * Sets this variable's current value.
    * <p>
    * This variable's listeners will be notified if this variable's value is changed.
    * </p>
    *
    * @param value the new value for this variable.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    */
   public boolean set(double value)
   {
      return set(value, true);
   }

   /**
    * Sets this variable's current value.
    *
    * @param value           the new value for this variable.
    * @param notifyListeners whether to notify this variable's listeners if this operation results in
    *                        changing this variable's current value.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    */
   public boolean set(double value, boolean notifyListeners)
   {
      if (this.value != value)
      {
         this.value = value;
         if (notifyListeners)
            notifyListeners();
         return true;
      }
      return false;
   }

   /**
    * Redirection to {@link #getDoubleValue()}.
    *
    * @return current value for this variable.
    */
   @Override
   public double getValueAsDouble()
   {
      return getDoubleValue();
   }

   /**
    * Sets this variable's current value.
    * <p>
    * Redirection to {@link #set(double, boolean)}.
    * </p>
    *
    * @param value the new value for this variable.
    */
   @Override
   public boolean setValueFromDouble(double value, boolean notifyListeners)
   {
      return set(value, notifyListeners);
   }

   /**
    * Retrieves this variable's current value as a long.
    *
    * @return long representing the current value using {@link Double#doubleToLongBits(double)}.
    */
   @Override
   public long getValueAsLongBits()
   {
      return Double.doubleToLongBits(value);
   }

   /**
    * Sets this variable's current value after converting the given long value.
    *
    * @param value converted to a double using {@link Double#longBitsToDouble(long)}.
    */
   @Override
   public boolean setValueFromLongBits(long value, boolean notifyListeners)
   {
      return set(Double.longBitsToDouble(value), notifyListeners);
   }

   /**
    * Sets this variable's value from the other variable once casted to {@code YoDouble}.
    * 
    * @param other the other {@code YoDouble} used to update this variable's value.
    * @throws ClassCastException if {@code other} cannot be casted as a {@code YoDouble}.
    */
   @Override
   public boolean setValue(YoVariable other, boolean notifyListeners)
   {
      return set(((YoDouble) other).getAsDouble(), notifyListeners);
   }

   /**
    * Returns the value of this variable as a string.
    * 
    * @return string representation of the current value using to {@link Double#toString(double)} when
    *         no format is provided. When a format is provided,
    *         {@link String#format(String, Object...)} is used.
    */
   @Override
   public String getValueAsString(String format)
   {
      return convertDoubleValueToString(format, value);
   }

   /**
    * Tries to parse the given string and set this variable's value using
    * {@link Double#parseDouble(String)}.
    */
   @Override
   public boolean parseValue(String valueAsString, boolean notifyListeners)
   {
      return set(Double.parseDouble(valueAsString), notifyListeners);
   }

   @Override
   public String convertDoubleValueToString(String format, double value)
   {
      if (format == null)
         return Double.toString(value);
      else
         return String.format(format, value);
   }

   /**
    * Assesses if this variable is equal to zero.
    *
    * @return {@code true} if this variable's value is {@code 0.0}.
    */
   @Override
   public boolean isZero()
   {
      return value == 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public YoDouble duplicate(YoRegistry newRegistry)
   {
      YoDouble duplicate = new YoDouble(getName(), getDescription(), newRegistry);
      duplicate.setVariableBounds(getLowerBound(), getUpperBound());
      duplicate.set(value);
      return duplicate;
   }

   /** {@inheritDoc} */
   @Override
   public String toString()
   {
      return String.format("%s: %s", getName(), value);
   }
}
