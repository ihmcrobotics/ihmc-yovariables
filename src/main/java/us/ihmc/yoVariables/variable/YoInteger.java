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
   private int val;

   /**
    * Create a new YoInteger. This will call {@link #YoInteger(String, String, YoRegistry)}
    * with the given name and registry and an empty description.
    *
    * @param name     String that uniquely identifies this YoInteger
    * @param registry YoVariableRegistry for this YoInteger to register itself to after initialization
    */
   public YoInteger(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new YoInteger. This will call {@link #YoInteger(String, String, YoRegistry)}
    * with the given values as well as set {@link #manualMaxScaling} and {@link #manualMaxScaling} to
    * the given values.
    *
    * @param name        String that uniquely identifies this YoInteger
    * @param description String that describes this YoInteger's purpose
    * @param registry    YoVariableRegistry for this YoInteger to register itself to after
    *                    initialization
    * @param minScaling  double to set manualMinScaling to
    * @param maxScaling  double to set manualMaxScaling to
    */
   public YoInteger(String name, String description, YoRegistry registry, double minScaling, double maxScaling)
   {
      this(name, description, registry);

      manualMinScaling = minScaling;
      manualMaxScaling = maxScaling;
   }

   /**
    * Create a new YoInteger. This will call its super YoVariable's {@link YoVariable(YoVariableType,
    * String, String, YoVariableRegistry)} with {@link YoVariableType#INTEGER} and the given values.
    *
    * @param name        String that uniquely identifies this YoInteger
    * @param description String that describes this YoInteger's purpose
    * @param registry    YoVariableRegistry for this YoInteger to register itself to after
    *                    initialization
    */
   public YoInteger(String name, String description, YoRegistry registry)
   {
      super(YoVariableType.INTEGER, name, description, registry);

      this.set(0);
   }

   /**
    * Calls {@link #set(int, boolean)} with value and true.
    *
    * @param value integer to set this YoInteger's internal integer state to
    */
   public void set(int value)
   {
      set(value, true);
   }

   /**
    * Sets this YoInteger to the given value.
    *
    * @param value           integer to set this YoInteger's internal integer state to
    * @param notifyListeners boolean determining whether or not to call
    *                        {@link #notifyVariableChangedListeners()}
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
      this.set(getIntegerValue() + 1);
   }

   /**
    * Sets this YoInteger to its current value minus one.
    */
   public void decrement()
   {
      this.set(getIntegerValue() - 1);
   }

   /**
    * Sets this YoInteger to its current value plus the given value.
    *
    * @param value integer to add to this YoInteger
    */
   public void add(int value)
   {
      this.set(getIntegerValue() + value);
   }

   /**
    * Sets this YoInteger to its current value minus the given value.
    *
    * @param value integer to subtract from this YoInteger
    */
   public void subtract(int value)
   {
      this.set(getIntegerValue() - value);
   }

   /**
    * Retrieves the value of this YoInteger.
    *
    * @return the internal integer value of this YoInteger
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
    * Set the value of this YoInteger using the given double, passed through
    * {@link #convertFromDoubleToInt(double)}.
    *
    * @param doubleValue     double to convert and set this YoInteger to
    * @param notifyListeners boolean determining whether or not to call
    *                        {@link #notifyVariableChangedListeners()}
    */
   @Override
   public void setValueFromDouble(double doubleValue, boolean notifyListeners)
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
   @Override
   public double getValueAsDouble()
   {
      return val;
   }

   /**
    * Returns String representation of this YoInteger.
    *
    * @return String representing this YoInteger and its current value as an integer
    */
   @Override
   public String toString()
   {
      return String.format("%s: %d", getName(), getIntegerValue());
   }

   /**
    * Retrieves this YoInteger's value as a long.
    *
    * @return return-casted long value of this YoInteger's internal integer value
    */
   @Override
   public long getValueAsLongBits()
   {
      return val;
   }

   /**
    * Sets the internal integer value of this YoInteger using the static integer cast of the passed
    * long value.
    *
    * @param value           long to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call
    *                        {@link #notifyVariableChangedListeners()}
    */
   @Override
   public void setValueFromLongBits(long value, boolean notifyListeners)
   {
      set((int) value, notifyListeners);
   }

   /**
    * Creates a new YoInteger with the same parameters as this one, and registers it to the passed
    * {@link YoRegistry}.
    *
    * @param newRegistry YoVariableRegistry to duplicate this YoInteger to
    * @return the newly created and registered YoInteger
    */
   @Override
   public YoInteger duplicate(YoRegistry newRegistry)
   {
      YoInteger retVar = new YoInteger(getName(), getDescription(), newRegistry, getManualScalingMin(), getManualScalingMax());
      retVar.set(getIntegerValue());
      return retVar;
   }

   /**
    * Assesses if this YoInteger is equal to zero.
    *
    * @return boolean if this YoInteger's internal integer value is equal to integer 0
    */
   @Override
   public boolean isZero()
   {
      return getIntegerValue() == 0;
   }

   @Override
   public int getValue()
   {
      return getIntegerValue();
   }
}
