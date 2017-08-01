package us.ihmc.yoVariables.variable;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Integer implementation of the YoVariable class.
 */
public class YoInteger extends YoVariable<YoInteger>
{
   private int val;

   public YoInteger(String name, YoVariableRegistry registry)
   {
      this(name, "", registry);
   }

   public YoInteger(String name, String description, YoVariableRegistry registry, double minScaling, double maxScaling)
   {
      this(name, description, registry);

      this.manualMinScaling = minScaling;
      this.manualMaxScaling = maxScaling;
   }

   /**
    * Create a new YoInteger. This will call its super YoVariable's {@link YoVariable(YoVariableType, String, String, YoVariableRegistry)}
    * with YoVariableType.INTEGER and the given values.
    *
    * @param name 
    * @param description
    * @param registry
    */
   public YoInteger(String name, String description, YoVariableRegistry registry)
   {
      super(YoVariableType.INTEGER, name, description, registry);

      this.set(0);
   }

   /**
    * Calls {@link #set(int, boolean)} with (value, true).
    *
    * @param value to set this YoInteger's internal integer state to
    */
   public void set(int value)
   {
      set(value, true);
   }

   /**
    * Sets this YoInteger to the given value.
    *
    * @param value to set this YoInteger's internal integer state to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    * @return boolean if the given value differed from the current value of this YoInteger
    */
   public boolean set(int value, boolean notifyListeners)
   {
      if (val != value)
      {
         val = value;
         if (notifyListeners)
         {
            notifyVariableChangedListeners();
         }
         return true;
      }
      return false;
   }

   /**
    * Sets this YoInteger to its current value plus one.
    */
   public void increment()
   {
      this.set(this.getIntegerValue() + 1);
   }

   /**
    * Sets this YoInteger to its current value minus one.
    */
   public void decrement()
   {
      this.set(this.getIntegerValue() - 1);
   }

   /**
    * Sets this YoInteger to its current value plus the given value.
    *
    * @param value to add to this YoInteger
    */
   public void add(int value)
   {
      this.set(this.getIntegerValue() + value);
   }

   /**
    * Sets this YoInteger to its current value minus the given value.
    *
    * @param value to subtract from this YoInteger
    */
   public void subtract(int value)
   {
      this.set(this.getIntegerValue() - value);
   }

   /**
    * Retrieves the value of this YoInteger.
    *
    * @return the internal integer state of this YoInteger
    */
   public int getIntegerValue()
   {
      return val;
   }

   /**
    * Check if the value contained by this YoInteger is equal to the given integer.
    *
    * @param value int for this YoInteger to be compared to
    * @return boolean if this YoInteger's value is the same as the passed value
    */
   public boolean valueEquals(int value)
   {
      return val == value;
   }

   /**
    * Set the value of this YoInteger using the given double, passed through {@link #convertFromDoubleToInt(double)}.
    *
    * @param doubleValue to convert and set this YoInteger to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    */
   @Override public void setValueFromDouble(double doubleValue, boolean notifyListeners)
   {
      set(convertFromDoubleToInt(doubleValue), notifyListeners);
   }

   /**
    * Returns the given double value converted to the YoInteger representation.
    *
    * @param doubleValue double to convert
    * @return rounded integer value representing the double
    */
   public int convertFromDoubleToInt(double doubleValue)
   {
      // Note: do not expect this to work well for very large values!
      return (int) Math.round(doubleValue);
   }

   /**
    * Retrieves this YoInteger's value as a double.
    *
    * @return return-casted double value of this YoInteger's internal integer value
    */
   @Override public double getValueAsDouble()
   {
      return val;
   }

   /**
    * Returns String representation of this YoInteger.
    *
    * @return a String representing this YoInteger and its current value as an integer
    */
   @Override public String toString()
   {
      return String.format("%s: %d", getName(), getIntegerValue());
   }

   /**
    * Appends the value of this YoInteger to the end of the given StringBuffer.
    *
    * @param stringBuffer StringBuffer to which the value will be appended
    */
   @Override public void getValueString(StringBuffer stringBuffer)
   {
      stringBuffer.append(val);
   }

   /**
    * Appends the YoInteger representation of the given double value to the given StringBuffer.
    *
    * @param stringBuffer StringBuffer to append to
    * @param doubleValue double value to convert to YoInteger representation
    */
   @Override public void getValueStringFromDouble(StringBuffer stringBuffer, double doubleValue)
   {
      stringBuffer.append(convertFromDoubleToInt(doubleValue));
   }

   /**
    * Retrieves this YoInteger's value as a long.
    *
    * @return return-casted long value of this YoInteger's internal integer value
    */
   @Override public long getValueAsLongBits()
   {
      return val;
   }

   /**
    * Sets the internal integer value of this YoInteger using the static integer cast of the passed long value.
    *
    * @param value long to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    */
   @Override public void setValueFromLongBits(long value, boolean notifyListeners)
   {
      set((int) value, notifyListeners);
   }

   /**
    * Creates a new YoInteger with the same parameters as this one, and registers it to the passed {@link YoVariableRegistry}.
    *
    * @param newRegistry YoVariableRegistry to duplicate this variable to
    * @return the newly created and registered YoInteger
    */
   @Override public YoInteger duplicate(YoVariableRegistry newRegistry)
   {
      YoInteger retVar = new YoInteger(getName(), getDescription(), newRegistry, getManualScalingMin(), getManualScalingMax());
      retVar.set(getIntegerValue());
      return retVar;
   }

   /**
    * Sets the interal value of this YoInteger to the current value of the passed YoInteger.
    *
    * @param value YoInteger value to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    * @return boolean whether or not internal state differed from the passed value
    */
   @Override public boolean setValue(YoInteger value, boolean notifyListeners)
   {
      return set(value.getIntegerValue(), notifyListeners);
   }

   /**
    * Assesses if this YoInteger is equal to zero.
    *
    * @return boolean if this YoInteger's internal integer value is equal to integer 0
    */
   @Override public boolean isZero()
   {
      return getIntegerValue() == 0;
   }
}
