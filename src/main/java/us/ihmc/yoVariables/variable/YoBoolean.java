package us.ihmc.yoVariables.variable;

import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Boolean implementation of a {@code YoVariable}.
 * 
 * @see YoVariable
 */
public class YoBoolean extends YoVariable implements BooleanProvider
{
   /**
    * Internal boolean value of this YoBoolean.
    */
   private boolean value;

   /**
    * Create a new {@code YoBoolean} and initializes to {@code false}.
    *
    * @param name     the name for this variable that can be used to retrieve it from a
    *                 {@link YoRegistry}.
    * @param registry initial parent registry for this variable.
    */
   public YoBoolean(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new {@code YoBoolean} and initializes to {@code false}.
    *
    * @param name        the name for this variable that can be used to retrieve it from a
    *                    {@link YoRegistry}.
    * @param description description of this variable's purpose.
    * @param registry    initial parent registry for this variable.
    * @see YoVariable#YoVariable(YoVariableType, String, String, YoRegistry)
    */
   public YoBoolean(String name, String description, YoRegistry registry)
   {
      super(YoVariableType.BOOLEAN, name, description, registry);
      this.set(false);
   }

   /**
    * Tests if the variable's current value is equal to the given boolean.
    *
    * @param value the query.
    * @return boolean if this variable's value is equal to the query.
    */
   public boolean valueEquals(boolean value)
   {
      return this.value == value;
   }

   /**
    * Retrieves the current boolean value of this variable.
    *
    * @return the internal boolean value of this variable.
    */
   @Override
   public boolean getValue()
   {
      return value;
   }

   /**
    * Retrieves the current boolean value of this variable.
    *
    * @return the internal boolean value of this variable.
    */
   public boolean getBooleanValue()
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
   public boolean set(boolean value)
   {
      return set(value, true);
   }

   /**
    * Sets this YoBoolean to the given value.
    *
    * @param value           the new value for this variable.
    * @param notifyListeners whether to notify this variable's listeners if this operation results in
    *                        changing this variable's current value.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    */
   public boolean set(boolean value, boolean notifyListeners)
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
    * Retrieves this variable's value as a double.
    *
    * @return {@code 1.0} if this variable is currently {@code true}, {@code 0.0} otherwise.
    */
   @Override
   public double getValueAsDouble()
   {
      return value ? 1.0 : 0.0;
   }

   /**
    * Sets this variable's value from the given double.
    *
    * @param value converted to @{@code true} if {@code value >= 0.5}, {@code false} otherwise.
    */
   @Override
   public boolean setValueFromDouble(double value, boolean notifyListeners)
   {
      return set(value >= 0.5, notifyListeners);
   }

   /**
    * Retrieves this YoBoolean's value as a long.
    *
    * @return {@code 1} if this variable is currently {@code true}, {@code 0} otherwise.
    */
   @Override
   public long getValueAsLongBits()
   {
      return value ? 1 : 0;
   }

   /**
    * Sets this variable's current value after converting the given long value.
    *
    * @param value converted to {@code true} if equal to {@code 1}, {@code false} otherwise.
    */
   @Override
   public boolean setValueFromLongBits(long value, boolean notifyListeners)
   {
      return set(value == 1, notifyListeners);
   }

   /**
    * Sets this variable's value from the other variable once casted to {@code YoBoolean}.
    * 
    * @param other the other {@code YoBoolean} used to update this variable's value.
    */
   @Override
   public boolean setValue(YoVariable other, boolean notifyListeners)
   {
      return set(((YoBoolean) other).getValue(), notifyListeners);
   }

   /**
    * Returns the value of this variable as a string.
    * 
    * @return string representation of the current value according to
    *         {@link Boolean#toString(boolean)}.
    */
   @Override
   public String getValueAsString(String format)
   {
      return Boolean.toString(value);
   }

   /**
    * Tries to parse the given string and set this variable's value using
    * {@link Boolean#parseBoolean(String)}.
    */
   @Override
   public boolean parseValue(String valueAsString, boolean notifyListeners)
   {
      return set(Boolean.parseBoolean(valueAsString), notifyListeners);
   }

   /**
    * Assesses if this variable is equal to zero.
    *
    * @return {@code true} if this variable's current value is {@code false}.
    */
   @Override
   public boolean isZero()
   {
      return !value;
   }

   /** {@inheritDoc} */
   @Override
   public YoBoolean duplicate(YoRegistry newRegistry)
   {
      YoBoolean duplicate = new YoBoolean(getName(), getDescription(), newRegistry);
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