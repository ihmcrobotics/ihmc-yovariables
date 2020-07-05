package us.ihmc.yoVariables.variable;

import us.ihmc.yoVariables.providers.LongProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Long implementation of the YoVariable class.
 * <p>
 * All abstract functions of YoVariable will be implemented using long type for interpretation.
 * Values will be interpreted, compared, and returned as longs rather than other native types.
 */
public class YoLong extends YoVariable<YoLong> implements LongProvider
{
   /**
    * Internal long value of this YoLong.
    */
   private long val;

   /**
    * Create a new YoLong. This will call {@link #YoLong(String, String, YoRegistry)} with the
    * given name and registry and an empty description.
    *
    * @param name     String that uniquely identifies this YoLong
    * @param registry YoVariableRegistry for this YoLong to register itself to after initialization
    */
   public YoLong(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new YoLong. This will call {@link #YoLong(String, String, YoRegistry)} with the
    * given values as well as set {@link #manualMinScaling} and {@link #manualMaxScaling} to the given
    * values.
    *
    * @param name        String that uniquely identifies this YoLong
    * @param description String that describes this YoLong's purpose
    * @param registry    YoVariableRegistry for this YoLong to register itself to after initialization
    * @param minScaling  double to set manualMinScaling to
    * @param maxScaling  double to set manualMaxScaling to
    */
   public YoLong(String name, String description, YoRegistry registry, double minScaling, double maxScaling)
   {
      this(name, description, registry);

      manualMinScaling = minScaling;
      manualMaxScaling = maxScaling;
   }

   /**
    * Create a new YoLong. This will call {@link YoVariable(YoVariableType, String, String,
    * YoVariableRegistry)} with {@link YoVariableType#LONG} and the given values.
    *
    * @param name
    * @param description
    * @param registry
    */
   public YoLong(String name, String description, YoRegistry registry)
   {
      super(YoVariableType.LONG, name, description, registry);

      this.set(0);
   }

   /**
    * Calls {@link #set(long, boolean)} with value and true.
    *
    * @param value long to set this YoInteger's internal integer state to
    */
   public void set(long value)
   {
      set(value, true);
   }

   /**
    * Sets this YoLong to the given value.
    *
    * @param value           long to set this YoLong's internal long state to
    * @param notifyListeners boolean determining whether or not to call
    *                        {@link #notifyVariableChangedListeners()}
    * @return boolean if the given value differed from the current value of this YoLong
    */
   public boolean set(long value, boolean notifyListeners)
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
      this.set(getLongValue() + 1);
   }

   /**
    * Sets this YoInteger to its current value minus one.
    */
   public void decrement()
   {
      this.set(getLongValue() - 1);
   }

   /**
    * Sets this YoLong to its current value plus the given value.
    *
    * @param value long to add to this YoLong
    */
   public void add(long value)
   {
      this.set(getLongValue() + value);
   }

   /**
    * Sets this YoLong to its current value minus the given value.
    *
    * @param value long to subtract from this YoLong
    */
   public void subtract(long value)
   {
      this.set(getLongValue() - value);
   }

   /**
    * Retrieves the value of this YoLong.
    *
    * @return the internal long value of this YoLong
    */
   public long getLongValue()
   {
      return val;
   }

   /**
    * Check if the value contained by this YoLong is equal to the given long.
    *
    * @param value long for this YoLong to be compared to
    * @return boolean if this YoLong's value is the same as the passed value
    */
   public boolean valueEquals(long value)
   {
      return val == value;
   }

   /**
    * Set the value of this YoLong using the given double, passed through
    * {@link #convertFromDoubleToLong(double)}.
    *
    * @param doubleValue     double to convert and set this YoLong to
    * @param notifyListeners boolean determining whether or not to call
    *                        {@link #notifyVariableChangedListeners()}
    */
   @Override
   public void setValueFromDouble(double doubleValue, boolean notifyListeners)
   {
      set(convertFromDoubleToLong(doubleValue), notifyListeners);
   }

   public long convertFromDoubleToLong(double doubleValue)
   {
      return Math.round(doubleValue);
   }

   /**
    * Retrieves this YoLong's value as a double.
    *
    * @return return-casted double value of this YoInteger's internal long value
    */
   @Override
   public double getValueAsDouble()
   {
      return val;
   }

   /**
    * Returns String representation of this YoLong.
    *
    * @return a String representing this YoLong and its current value as a long
    */
   @Override
   public String toString()
   {
      return String.format("%s: %d", getName(), getLongValue());
   }

   /**
    * Appends the value of this variable to the end of the given StringBuffer.
    *
    * @param stringBuffer StringBuffer to which the value will be appended
    */
   @Override
   public void getValueString(StringBuffer stringBuffer)
   {
      stringBuffer.append(val);
   }

   /**
    * Appends the YoLong representation of the given double value to the given StringBuffer.
    *
    * @param stringBuffer StringBuffer to append to
    * @param doubleValue  double value to convert to YoLong representation
    */
   @Override
   public void getValueStringFromDouble(StringBuffer stringBuffer, double doubleValue)
   {
      stringBuffer.append(convertFromDoubleToLong(doubleValue));
   }

   /**
    * Retrieves this YoLong's value as a long.
    * <p>
    * Effectively equivalent to {@link #getLongValue()}.
    *
    * @return internal long value of this YoInteger
    */
   @Override
   public long getValueAsLongBits()
   {
      return val;
   }

   /**
    * Sets the internal long value of this YoLong using the passed long value.
    * <p>
    * Effectively equivalent to {@link #set(long, boolean)}.
    *
    * @param value           long to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call
    *                        {@link #notifyVariableChangedListeners()}
    */
   @Override
   public void setValueFromLongBits(long value, boolean notifyListeners)
   {
      set(value, notifyListeners);
   }

   /**
    * Creates a new YoLong with the same parameters as this one, and registers it to the passed
    * {@link YoRegistry}.
    *
    * @param newRegistry YoVariableRegistry to duplicate this YoLong to
    * @return the newly created and registered YoLong
    */
   @Override
   public YoLong duplicate(YoRegistry newRegistry)
   {
      YoLong retVar = new YoLong(getName(), getDescription(), newRegistry, getManualScalingMin(), getManualScalingMax());
      retVar.set(getLongValue());
      return retVar;
   }

   /**
    * Sets the internal value of this YoLong to the current value of the passed YoLong.
    *
    * @param value           YoLong value to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call
    *                        {@link #notifyVariableChangedListeners()}
    * @return boolean whether or not internal state differed from the passed value
    */
   @Override
   public boolean setValue(YoLong value, boolean notifyListeners)
   {
      return set(value.getLongValue(), notifyListeners);
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

   @Override
   public long getValue()
   {
      return getLongValue();
   }
}
