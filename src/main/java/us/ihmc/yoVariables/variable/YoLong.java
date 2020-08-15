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

import us.ihmc.yoVariables.providers.LongProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Long implementation of a {@code YoVariable}.
 * 
 * @see YoVariable
 */
public class YoLong extends YoVariable implements LongProvider
{
   /**
    * Internal long value of this YoLong.
    */
   private long value;

   /**
    * Create a new {@code YoLong} and initializes to {@code 0}.
    *
    * @param name     the name for this variable that can be used to retrieve it from a
    *                 {@link YoRegistry}.
    * @param registry initial parent registry for this variable.
    */
   public YoLong(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new {@code YoLong} and initializes to {@code 0}.
    *
    * @param name        the name for this variable that can be used to retrieve it from a
    *                    {@link YoRegistry}.
    * @param description description of this variable's purpose.
    * @param registry    initial parent registry for this variable.
    * @see YoVariable#YoVariable(YoVariableType, String, String, YoRegistry)
    */
   public YoLong(String name, String description, YoRegistry registry)
   {
      super(YoVariableType.LONG, name, description, registry);
      this.set(0);
   }

   /**
    * Sets this YoInteger to its current value plus one.
    */
   public void increment()
   {
      this.set(value + 1);
   }

   /**
    * Sets this YoInteger to its current value minus one.
    */
   public void decrement()
   {
      this.set(value - 1);
   }

   /**
    * Sets this YoLong to its current value plus the given value.
    *
    * @param value long to add to this YoLong
    */
   public void add(long value)
   {
      this.set(this.value + value);
   }

   /**
    * Sets this YoLong to its current value minus the given value.
    *
    * @param value long to subtract from this YoLong
    */
   public void subtract(long value)
   {
      this.set(this.value - value);
   }

   /**
    * Retrieves the value of this YoLong.
    *
    * @return the internal long value of this YoLong
    */
   @Override
   public long getValue()
   {
      return value;
   }

   /**
    * Retrieves the value of this YoLong.
    *
    * @return the internal long value of this YoLong
    */
   public long getLongValue()
   {
      return value;
   }

   /**
    * Tests if the variable's current value is equal to the given long.
    *
    * @param value the query.
    * @return boolean if this variable's value is equal to the query.
    */
   public boolean valueEquals(long value)
   {
      return this.value == value;
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
   public boolean set(long value)
   {
      return set(value, true);
   }

   /**
    * Sets this YoLong to the given value.
    *
    * @param value           the new value for this variable.
    * @param notifyListeners whether to notify this variable's listeners if this operation results in
    *                        changing this variable's current value.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    */
   public boolean set(long value, boolean notifyListeners)
   {
      if (this.value != value)
      {
         this.value = value;
         if (notifyListeners)
         {
            notifyListeners();
         }
         return true;
      }
      return false;
   }

   /**
    * Retrieves this YoLong's value as a double.
    *
    * @return casted double value of this variable's current value.
    */
   @Override
   public double getValueAsDouble()
   {
      return value;
   }

   /**
    * Sets this variable's value from the given double.
    *
    * @param value rounded to a long.
    */
   @Override
   public boolean setValueFromDouble(double value, boolean notifyListeners)
   {
      return set(Math.round(value), notifyListeners);
   }

   /**
    * Retrieves this YoLong's value as a long.
    * <p>
    * Redirection to {@link #getLongValue()}.
    * </p>
    *
    * @return internal long value of this YoInteger
    */
   @Override
   public long getValueAsLongBits()
   {
      return getLongValue();
   }

   /**
    * Sets this variable's current value after converting the given long value.
    * <p>
    * Redirection to {@link #set(long, boolean)}.
    * </p>
    *
    * @param value the new value for this variable.
    */
   @Override
   public boolean setValueFromLongBits(long value, boolean notifyListeners)
   {
      return set(value, notifyListeners);
   }

   /**
    * Sets this variable's value from the other variable once casted to {@code YoLong}.
    *
    * @param other the other {@code YoLong} used to update this variable's value.
    * @throws ClassCastException if {@code other} cannot be casted as a {@code YoLong}.
    */
   @Override
   public boolean setValue(YoVariable other, boolean notifyListeners)
   {
      return set(((YoLong) other).getValue(), notifyListeners);
   }

   /**
    * Returns the value of this variable as a string.
    *
    * @return string representation of the current value according to {@link Long#toString(long)}.
    */
   @Override
   public String getValueAsString(String format)
   {
      return Long.toString(value);
   }

   /**
    * Tries to parse the given string and set this variable's value using
    * {@link Long#parseLong(String)}.
    */
   @Override
   public boolean parseValue(String valueAsString, boolean notifyListeners)
   {
      return set(Long.parseLong(valueAsString), notifyListeners);
   }

   @Override
   public String convertDoubleValueToString(String format, double value)
   {
      return Long.toString((long) value);
   }

   /**
    * Assesses if this YoLong is equal to zero.
    *
    * @return boolean if this YoLong's internal long value is equal to long 0
    */
   @Override
   public boolean isZero()
   {
      return getLongValue() == 0;
   }

   /** {@inheritDoc} */
   @Override
   public YoLong duplicate(YoRegistry newRegistry)
   {
      YoLong duplicate = new YoLong(getName(), getDescription(), newRegistry);
      duplicate.setVariableBounds(getLowerBound(), getUpperBound());
      duplicate.set(getLongValue());
      return duplicate;
   }

   /** {@inheritDoc} */
   @Override
   public String toString()
   {
      return String.format("%s: %d", getName(), getLongValue());
   }
}
