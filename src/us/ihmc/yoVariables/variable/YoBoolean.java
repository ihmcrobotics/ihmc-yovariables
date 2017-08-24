package us.ihmc.yoVariables.variable;

import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Boolean implementation of the YoVariable class.
 *
 * <p>All abstract functions of YoVariable will be implemented using boolean type for interpretation.
 * Values will be interpreted, compared, and returned as booleans rather than other native types.
 */
public class YoBoolean extends YoVariable<YoBoolean> implements BooleanProvider
{
   /**
    * Internal boolean value of this YoLong.
    */
   private boolean val;

   /**
    * Create a new YoBoolean. This will call {@link #YoBoolean(String, String, YoVariableRegistry)} with the given name and
    * registry and an empty description.
    *
    * @param name String that uniquely identifies this YoBoolean
    * @param registry YoVariableRegistry for this YoBoolean to register itself to after initialization
    */
   public YoBoolean(String name, YoVariableRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new YoBoolean. This will call its super YoVariable's {@link YoVariable(YoVariableType, String, String, YoVariableRegistry)}
    * with {@link YoVariableType#BOOLEAN} and the given values.
    *
    * @param name String that uniquely identifies this YoBoolean
    * @param description String that describes this YoBoolean's purpose
    * @param registry YoVariableRegistry for this YoBoolean to register itself to after initialization
    */
   public YoBoolean(String name, String description, YoVariableRegistry registry)
   {
      super(YoVariableType.BOOLEAN, name, description, registry);

      this.stepSize = 1.0;
      this.set(false);
   }

   /**
    * Check if the value contained by this YoBoolean is equal to the given boolean.
    *
    * @param value long for this YoLong to be compared to
    * @return boolean if this YoLong's value is the same as the passed value
    */
   public boolean valueEquals(boolean value)
   {
      return (this.val == value);
   }

   /**
    * Retrieve the boolean value of this YoBoolean.
    *
    * @return the internal boolean value of this YoBoolean
    */
   public boolean getBooleanValue()
   {
      return val;
   }

   /**
    * Calls {@link #set(boolean, boolean)} with value and true.
    *
    * @param value boolean to set this YoBoolean's internal integer state to
    */
   public void set(boolean value)
   {
      set(value, true);
   }

   /**
    * Sets this YoBoolean to the given value.
    *
    * @param value long to set this YoBoolean's internal boolean state to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    * @return boolean if the given value differed from the current value of this YoBoolean
    */
   public boolean set(boolean value, boolean notifyListeners)
   {
      if (val != value)
      {
         this.val = value;
         if (notifyListeners)
         {
            notifyVariableChangedListeners();
         }
         return true;
      }
      return false;
   }

   /**
    * Set the value of this YoBoolean using the given double, passed through {@link #convertDoubleToBoolean(double)}.
    *
    * @param value boolean to convert and set this YoBoolean to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    */
   @Override public void setValueFromDouble(double value, boolean notifyListeners)
   {
      set(convertDoubleToBoolean(value), notifyListeners);
   }

   /**
    * Converts the given value to a boolean representation.
    *
    * @param value double to convert
    * @return boolean effectively boolean of rounded value (0 or 1)
    */
   public boolean convertDoubleToBoolean(double value)
   {
      if (value >= 0.5)
         return true;
      else
         return false;
   }

   /**
    * Retrieves this YoBoolean's value as a double.
    *
    * @return double of integer value of internal boolean value (0.0 or 1.0)
    */
   @Override public double getValueAsDouble()
   {
      double returnValue = 0.0;

      if (getBooleanValue())
         returnValue = 1.0;
      else
         returnValue = 0.0;

      return returnValue;
   }

   /**
    * Returns String representation of this YoBoolean.
    *
    * @return String representing this YoBoolean and its current value as a boolean
    */
   @Override public String toString()
   {
      return String.format("%s: %s", getName(), getBooleanValue());
   }

   /**
    * Appends the value of this variable to the end of the given StringBuffer.
    *
    * @param stringBuffer StringBuffer to which the value will be appended
    */
   @Override public void getValueString(StringBuffer stringBuffer)
   {
      stringBuffer.append(val);
   }

   /**
    * Appends the YoBoolean representation of the given double value to the given StringBuffer.
    *
    * @param stringBuffer StringBuffer to append to
    * @param doubleValue double value to convert to YoBoolean representation
    */
   @Override public void getValueStringFromDouble(StringBuffer stringBuffer, double doubleValue)
   {
      stringBuffer.append(convertDoubleToBoolean(doubleValue));
   }

   /**
    * Retrieves this YoBoolean's value as a long.
    *
    * @return long value of internal boolean state (0 or 1)
    */
   @Override public long getValueAsLongBits()
   {
      return getBooleanValue() ? 1 : 0;
   }

   /**
    * Sets the internal long value of this YoLong using the passed long value.
    *
    * <p>Results in being set to true if value is 1, or false otherwise.
    *
    * @param value long to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    */
   @Override public void setValueFromLongBits(long value, boolean notifyListeners)
   {
      set(value == 1, notifyListeners);
   }

   /**
    * Creates a new YoBoolean with the same parameters as this one, and registers it to the passed {@link YoVariableRegistry}.
    *
    * @param newRegistry YoVariableRegistry to duplicate this YoBoolean to
    * @return the newly created and registered YoBoolean
    */
   @Override public YoBoolean duplicate(YoVariableRegistry newRegistry)
   {
      YoBoolean newVar = new YoBoolean(getName(), getDescription(), newRegistry);
      newVar.set(getBooleanValue());
      return newVar;
   }

   /**
    * Sets the internal value of this YoBoolean to the current value of the passed YoBoolean.
    *
    * @param value YoBoolean value to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    * @return boolean whether or not internal state differed from the passed value
    */
   @Override public boolean setValue(YoBoolean value, boolean notifyListeners)
   {
      return set(value.getBooleanValue(), notifyListeners);
   }

   /**
    * Assesses if this YoBoolean is equal to zero.
    *
    * <p>Returns true if this YoBoolean's value is false, and false if otherwise.
    *
    * @return boolean inverse of this YoBoolean's internal boolean value
    */
   @Override public boolean isZero()
   {
      return !getBooleanValue();
   }
   
   @Override
   public boolean getValue()
   {
      return getBooleanValue();
   }

}