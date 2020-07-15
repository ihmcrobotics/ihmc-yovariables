package us.ihmc.yoVariables.variable;

import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Integer implementation of the YoVariable class.
 * <p>
 * All abstract functions of YoVariable will be implemented using integer type for interpretation.
 * Values will be interpreted, compared, and returned as integers rather than other native types.
 */
public class YoInteger extends YoVariable implements IntegerProvider
{
   /**
    * Internal integer value of this YoInteger.
    */
   private int value;

   /**
    * Create a new {@code YoInteger} and initializes to {@code 0}.
    *
    * @param name     the name for this variable that can be used to retrieve it from a
    *                 {@link YoRegistry}.
    * @param registry initial parent registry for this variable.
    */
   public YoInteger(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new {@code YoInteger} and initializes to {@code 0}.
    *
    * @param name        the name for this variable that can be used to retrieve it from a
    *                    {@link YoRegistry}.
    * @param description description of this variable's purpose.
    * @param registry    initial parent registry for this variable.
    * @see YoVariable#YoVariable(YoVariableType, String, String, YoRegistry)
    */
   public YoInteger(String name, String description, YoRegistry registry)
   {
      super(YoVariableType.INTEGER, name, description, registry);
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
    * Sets this YoInteger to its current value plus the given value.
    *
    * @param value integer to add to this YoInteger.
    */
   public void add(int value)
   {
      this.set(this.value + value);
   }

   /**
    * Sets this YoInteger to its current value minus the given value.
    *
    * @param value integer to subtract from this YoInteger.
    */
   public void sub(int value)
   {
      this.set(this.value - value);
   }

   /**
    * Tests if the variable's current value is equal to the given integer.
    *
    * @param value the query.
    * @return boolean if this variable's value is equal to the query.
    */
   public boolean valueEquals(int value)
   {
      return this.value == value;
   }

   /**
    * Retrieves the current integer value of this variable.
    *
    * @return the internal integer value of this variable.
    */
   @Override
   public int getValue()
   {
      return value;
   }

   /**
    * Retrieves the current integer value of this variable.
    *
    * @return the internal integer value of this variable.
    */
   public int getIntegerValue()
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
   public boolean set(int value)
   {
      return set(value, true);
   }

   /**
    * Sets this YoInteger to the given value.
    *
    * @param value           the new value for this variable.
    * @param notifyListeners whether to notify this variable's listeners if this operation results in
    *                        changing this variable's current value.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    */
   public boolean set(int value, boolean notifyListeners)
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
    * Retrieves this YoInteger's value as a double.
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
    * @param value rounded and then casted to an integer.
    */
   @Override
   public boolean setValueFromDouble(double value, boolean notifyListeners)
   {
      return set((int) Math.round(value), notifyListeners);
   }

   /**
    * Retrieves this YoInteger's value as a long.
    *
    * @return casted long value of this variable's current value.
    */
   @Override
   public long getValueAsLongBits()
   {
      return value;
   }

   /**
    * Sets this variable's current value after converting the given long value.
    *
    * @param value long to set this variable's value to after casted to an integer.
    */
   @Override
   public boolean setValueFromLongBits(long value, boolean notifyListeners)
   {
      return set((int) value, notifyListeners);
   }

   /**
    * Sets this variable's value from the other variable once casted to {@code YoInteger}.
    * 
    * @param other the other {@code YoInteger} used to update this variable's value.
    */
   @Override
   public boolean setValue(YoVariable other, boolean notifyListeners)
   {
      return set(((YoInteger) other).getValue(), notifyListeners);
   }

   /**
    * Returns the value of this variable as a string.
    * 
    * @return string representation of the current value according to {@link Integer#toString(int)}.
    */
   @Override
   public String getValueAsString(String format)
   {
      return Integer.toString(value);
   }

   /**
    * Tries to parse the given string and set this variable's value using
    * {@link Integer#parseInt(String)}.
    */
   @Override
   public boolean parseValue(String valueAsString, boolean notifyListeners)
   {
      return set(Integer.parseInt(valueAsString), notifyListeners);
   }

   /**
    * Assesses if this variable is equal to zero.
    *
    * @return {@code true} if this variable's value is {@code 0}.
    */
   @Override
   public boolean isZero()
   {
      return value == 0;
   }

   /** {@inheritDoc} */
   @Override
   public YoInteger duplicate(YoRegistry newRegistry)
   {
      YoInteger duplicate = new YoInteger(getName(), getDescription(), newRegistry);
      duplicate.setVariableBounds(getLowerBound(), getUpperBound());
      duplicate.set(value);
      return duplicate;
   }

   /** {@inheritDoc} */
   @Override
   public String toString()
   {
      return String.format("%s: %d", getName(), getIntegerValue());
   }
}
