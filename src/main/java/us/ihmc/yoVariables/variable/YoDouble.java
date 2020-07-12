package us.ihmc.yoVariables.variable;

import org.apache.commons.math3.util.Precision;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Double implementation of the YoVariable class.
 * <p>
 * All abstract functions of YoVariable will be implemented using double type for interpretation.
 * Values will be interpreted, compared, and returned as doubles rather than other native types.
 */
public class YoDouble extends YoVariable implements DoubleProvider
{
   private double val;

   /**
    * Create a new YoDouble. This will call {@link #YoDouble(String, String, YoRegistry)} with the
    * given name and registry and an empty description.
    *
    * @param name     String uniquely identifying this YoDouble
    * @param registry YoRegistry for this YoDouble to register itself to after initialization
    */
   public YoDouble(String name, YoRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new YoDouble. This will call {@link #YoDouble(String, String, YoRegistry)} with the
    * given values as well as set {@link #manualMinScaling} and {@link #manualMaxScaling} to the given
    * values.
    *
    * @param name        String uniquely identifying this YoDouble
    * @param description String describing this YoDouble's purpose
    * @param registry    YoRegistry for this YoDouble to register itself to after initialization
    * @param minScaling  double to set manualMinScaling to
    * @param maxScaling  double to set manualMaxScaling to
    */
   public YoDouble(String name, String description, YoRegistry registry, double minScaling, double maxScaling)
   {
      this(name, description, registry);
      setVariableBounds(minScaling, maxScaling);
   }

   /**
    * Create a new YoDouble. This will call {@link YoVariable(String, String, YoRegistry)} with
    * {@link YoVariableType#DOUBLE} and the given values.
    *
    * @param name        name to be used for all references of this variable by SCS
    * @param description A short description of this variable
    * @param registry    YoRegistry with which this variable is to be registered
    */
   public YoDouble(String name, String description, YoRegistry registry)
   {
      super(YoVariableType.DOUBLE, name, description, registry);

      this.set(0.0);
   }

   /**
    * Returns String representation of this YoDouble.
    *
    * @return String representing this YoDouble and its current value as a double
    */
   @Override
   public String toString()
   {
      return String.format("%s: %s", getName(), getDoubleValue());
   }

   /**
    * Assesses if this YoDouble's value is a NaN.
    *
    * @return boolean return of Double.isNaN on this YoDouble's internal double state
    */
   public boolean isNaN()
   {
      return Double.isNaN(val);
   }

   /**
    * Sets this YoDouble to its current value plus the current value of the given YoDouble.
    *
    * @param variable YoDouble whose value should be added to this YoDouble
    */
   public void add(YoDouble variable)
   {
      this.set(getDoubleValue() + variable.getDoubleValue());
   }

   /**
    * Sets this YoDouble to its current value minus the current value of the given YoDouble.
    *
    * @param variable YoDouble whose value should be subtracted from this YoDouble
    */
   public void sub(YoDouble variable)
   {
      this.set(getDoubleValue() - variable.getDoubleValue());
   }

   /**
    * Sets this YoDouble to its current value minus the given value.
    *
    * @param value double to subtract from this YoDouble
    */
   public void sub(double value)
   {
      this.set(getDoubleValue() - value);
   }

   /**
    * Sets this YoDouble to its current value plus the given value.
    *
    * @param value double to add to this YoDouble
    */
   public void add(double value)
   {
      this.set(getDoubleValue() + value);
   }

   /**
    * Sets this YoDouble to its current value multiplied by the given value.
    *
    * @param value double to multiply this YoDouble by
    */
   public void mul(double value)
   {
      this.set(getDoubleValue() * value);
   }

   /**
    * Sets this YoDouble to its current value multiplied by the current value of the given YoDouble.
    *
    * @param value YoDouble whose value should be used to multiply this YoDouble's value by
    */
   public void mul(YoDouble value)
   {
      this.set(getDoubleValue() * value.getDoubleValue());
   }

   /**
    * Check if the value contained by this YoDouble is equal to the given double.
    *
    * @param value double for this YoDouble to be compared to
    * @return boolean if this YoDouble's value is the same as the passed value
    */
   public boolean valueEquals(double value)
   {
      return val == value;
   }

   /**
    * Retrieves the value of this YoDouble.
    *
    * @return the internal double value of this YoDouble
    */
   public double getDoubleValue()
   {
      return val;
   }

   /**
    * Calls {@link #set(double, boolean)} with value and true.
    *
    * @param value long to set this YoDouble's internal double state to
    */
   public void set(double value)
   {
      set(value, true);
   }

   /**
    * Sets this YoDouble to the given value.
    *
    * @param value           double to set this YoDouble's internal long state to
    * @param notifyListeners boolean determining whether or not to call
    *                        {@link #notifyListeners()}
    * @return boolean if the given value differed from the current value of this YoDouble
    */
   public boolean set(double value, boolean notifyListeners)
   {
      if (val != value)
      {
         val = value;
         if (notifyListeners)
         {
            notifyListeners();
         }
         return true;
      }
      return false;
   }

   /**
    * Retrieves this YoDouble's value as a double.
    * <p>
    * Effectively equivalent to {@link #getDoubleValue()}.
    *
    * @return internal double value of this YoDouble.
    */
   @Override
   public double getValueAsDouble()
   {
      return getDoubleValue();
   }

   /**
    * Set the value of this YoDouble using the given double.
    * <p>
    * Effectively equivalent to {@link #set(double, boolean)}.
    *
    * @param value           double to set this YoDouble to
    * @param notifyListeners boolean determining whether or not to call
    *                        {@link #notifyListeners()}
    */
   @Override
   public void setValueFromDouble(double value, boolean notifyListeners)
   {
      set(value, notifyListeners);
   }

   /**
    * Retrieves this YoDouble's value as a long.
    *
    * @return long representing this YouDouble's internal long value passed through
    *         {@link Double#doubleToLongBits(double)}
    */
   @Override
   public long getValueAsLongBits()
   {
      return Double.doubleToLongBits(val);
   }

   /**
    * Sets the internal double value of this YoDouble using the passed long value.
    * <p>
    * Passes the given value through {@link Double#longBitsToDouble(long)}.
    *
    * @param value           long to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call
    *                        {@link #notifyListeners()}
    */
   @Override
   public void setValueFromLongBits(long value, boolean notifyListeners)
   {
      set(Double.longBitsToDouble(value), notifyListeners);
   }

   /**
    * Creates a new YoDouble with the same parameters as this one, and registers it to the passed
    * {@link YoRegistry}.
    *
    * @param newRegistry YoRegistry to duplicate this YoDouble to
    * @return the newly created and registered YoDouble
    */
   @Override
   public YoDouble duplicate(YoRegistry newRegistry)
   {
      YoDouble retVar = new YoDouble(getName(), getDescription(), newRegistry, getLowerBound(), getUpperBound());
      retVar.set(val);
      return retVar;
   }

   @Override
   public boolean setValue(YoVariable other, boolean notifyListeners)
   {
      return set(((YoDouble) other).getValue(), notifyListeners);
   }

   /**
    * Sets this YoDouble's internal double value to {@link Double#NaN}.
    */
   public void setToNaN()
   {
      this.set(Double.NaN);

   }

   /**
    * Assesses if this YoDouble is equal to zero.
    *
    * @return boolean if this YoDouble's internal double value is equal to double 0.0
    */
   @Override
   public boolean isZero()
   {
      return Precision.equals(0.0, getDoubleValue(), 1);
   }

   @Override
   public double getValue()
   {
      return getDoubleValue();
   }

   @Override
   public String getValueAsString(String format)
   {
      if (format == null)
         return Double.toString(val);
      else
         return String.format(format, val);
   }
}
