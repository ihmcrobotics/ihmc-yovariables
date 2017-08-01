package us.ihmc.yoVariables.variable;

import org.apache.commons.math3.util.Precision;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.text.FieldPosition;
import java.text.NumberFormat;

/**
 * Title:        Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 *
 * <p>YoVariables provide a simple, convenient mechanism for storing and manipulating robot data.  While each
 * essentially contains a double value YoVariables are designed for integration into the SCS GUI.  Once registered,
 * a variable will automatically become available to the GUI for graphing, modification and other data manipulation.
 * Historical values of all registered YoVariables are stored in the DataBuffer which may be exported for later use.</p>
 * <p>
 */
public class YoDouble extends YoVariable<YoDouble>
{
   private static final java.text.NumberFormat DOUBLE_FORMAT = new java.text.DecimalFormat(" 0.00000;-0.00000");
   private static final FieldPosition FIELD_POSITION = new FieldPosition(NumberFormat.INTEGER_FIELD);

   private double val;

   /**
    * Create a new YoDouble. This will call {@link #YoDouble(String, String, YoVariableRegistry)} with the given name and registry
    * and an empty description.
    *
    * @param name String uniquely identifying this YoDouble
    * @param registry YoVariableRegistry for this YoDouble to register itself to after initialization
    */
   public YoDouble(String name, YoVariableRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Create a new YoDouble. This will call {@link #YoDouble(String, String, YoVariableRegistry)} with the given values
    * as well as set {@link #manualMinScaling} and {@link #manualMaxScaling} to the given values.
    *
    * @param name String uniquely identifying this YoDouble
    * @param description String describing this YoDouble's purpose
    * @param registry YoVariableRegistry for this YoDouble to register itself to after initialization
    * @param minScaling double to set manualMinScaling to
    * @param maxScaling double to set manualMaxScaling to
    */
   public YoDouble(String name, String description, YoVariableRegistry registry, double minScaling, double maxScaling)
   {
      this(name, description, registry);

      this.manualMinScaling = minScaling;
      this.manualMaxScaling = maxScaling;
   }

   /**
    * Create a new YoDouble. This will call {@link YoVariable(String, String, YoVariableRegistry)} with
    * {@link YoVariableType#DOUBLE} and the given values.
    *
    * @param name name to be used for all references of this variable by SCS
    * @param description A short description of this variable
    * @param registry YoVariableRegistry with which this variable is to be registered
    */
   public YoDouble(String name, String description, YoVariableRegistry registry)
   {
      super(YoVariableType.DOUBLE, name, description, registry);

      this.set(0.0);
   }

   /**
    * Returns String representation of this YoDouble.
    *
    * @return a String representing this YoDouble and its current value as a double
    */
   @Override public String toString()
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
      this.set(this.getDoubleValue() + variable.getDoubleValue());
   }

   /**
    * Sets this YoDouble to its current value minus the current value of the given YoDouble.
    *
    * @param variable YoDouble whose value should be subtracted from this YoDouble
    */
   public void sub(YoDouble variable)
   {
      this.set(this.getDoubleValue() - variable.getDoubleValue());
   }

   /**
    * Sets this YoDouble to its current value minus the given value.
    *
    * @param value double to subtract from this YoDouble
    */
   public void sub(double value)
   {
      this.set(this.getDoubleValue() - value);
   }

   /**
    * Sets this YoDouble to its current value plus the given value.
    *
    * @param value double to add to this YoDouble
    */
   public void add(double value)
   {
      this.set(this.getDoubleValue() + value);
   }

   /**
    * Sets this YoDouble to its current value multiplied by the given value.
    *
    * @param value double to multiply this YoDouble by
    */
   public void mul(double value)
   {
      this.set(this.getDoubleValue() * value);
   }

   /**
    * Sets this YoDouble to its current value multiplied by the current value of the given YoDouble.
    *
    * @param value YoDouble whose value should be used to multiply this YoDouble's value by
    */
   public void mul(YoDouble value)
   {
      this.set(this.getDoubleValue() * value.getDoubleValue());
   }

   /**
    * Check if the value contained by this YoDouble is equal to the given double.
    *
    * @param value double for this YoDouble to be compared to
    * @return boolean if this YoDouble's value is the same as the passed value
    */
   public boolean valueEquals(double value)
   {
      return (val == value);
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
    * @param value double to set this YoDouble's internal long state to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    * @return boolean if the given value differed from the current value of this YoDouble
    */
   public boolean set(double value, boolean notifyListeners)
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
    * Appends the value of this variable to the end of the given StringBuffer.
    *
    * @param stringBuffer StringBuffer to which the value will be appended
    */
   @Override public void getValueString(StringBuffer stringBuffer)
   {
      getValueStringFromDouble(stringBuffer, val);
   }

   /**
    * Appends the YoDouble representation of the given double value to the given StringBuffer.
    *
    * @param stringBuffer StringBuffer to append to
    * @param doubleValue double value to convert to YoDouble representation
    */
   @Override public void getValueStringFromDouble(StringBuffer stringBuffer, double doubleValue)
   {
      DOUBLE_FORMAT.format(doubleValue, stringBuffer, FIELD_POSITION); // Add the variable value to it
   }

   /**
    * Retrieves this YoDouble's value as a double.
    *
    * <p>Effectively equivalent to {@link #getDoubleValue()}.
    *
    * @return internal double value of this YoDouble.
    */
   @Override public double getValueAsDouble()
   {
      return getDoubleValue();
   }

   /**
    * Set the value of this YoDouble using the given double.
    *
    * <p>Effectively equivalent to {@link #set(double, boolean)}.
    *
    * @param value double to set this YoDouble to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    */
   @Override public void setValueFromDouble(double value, boolean notifyListeners)
   {
      set(value, notifyListeners);
   }

   /**
    * Retrieves this YoDouble's value as a long.
    *
    * @return long representing this YouDouble's internal long value passed through {@link Double#doubleToLongBits(double)}
    */
   @Override public long getValueAsLongBits()
   {
      return Double.doubleToLongBits(val);
   }

   /**
    * Sets the internal double value of this YoDouble using the passed long value.
    *
    * <p>Passes the given value through {@link Double#longBitsToDouble(long)}.
    *
    * @param value long to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    */
   @Override public void setValueFromLongBits(long value, boolean notifyListeners)
   {
      set(Double.longBitsToDouble(value), notifyListeners);
   }

   /**
    * Creates a new YoDouble with the same parameters as this one, and registers it to the passed {@link YoVariableRegistry}.
    *
    * @param newRegistry YoVariableRegistry to duplicate this YoDouble to
    * @return the newly created and registered YoDouble
    */
   @Override public YoDouble duplicate(YoVariableRegistry newRegistry)
   {
      YoDouble retVar = new YoDouble(getName(), getDescription(), newRegistry, getManualScalingMin(), getManualScalingMax());
      retVar.set(val);
      return retVar;
   }

   /**
    * Sets the internal value of this YoDouble to the current value of the passed YoDouble.
    *
    * @param value YoDouble value to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    * @return boolean whether or not internal state differed from the passed value
    */
   @Override public boolean setValue(YoDouble value, boolean notifyListeners)
   {
      return set(value.getDoubleValue(), notifyListeners);
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
   @Override public boolean isZero()
   {
      return Precision.equals(0.0, getDoubleValue(), 1);
   }

   //   NOTE: JEP October 30, 2010:
   //   The following is very useful for debugging things so please do not delete!
   //   I should probably use the change listener stuff instead, but this is nice for eavesdropping
   //   to catch when a variable changes or in order to compare two runs that should be identical
   //   to discover the first time their YoVariables differ...
   //
   //   private static boolean startDisplaying = false;
   //   private static boolean stopDisplaying = false;
   //   private static YoDouble time;
   //   private static PrintWriter writer;
   //
   //   private void setAndLogToAFile(double value)
   //   {
   //      if ((time == null) && (this.getName().equals("t")))
   //      {
   //         System.out.println("found time");
   //         time = this;
   //
   //
   //         try
   //         {
   //            writer = new PrintWriter("run.txt");
   //         } catch (FileNotFoundException e)
   //         {
   //
   //         }
   //      }
   //
   //      if ((time != null) && (time.getDoubleValue() >= 1.656-1e-7)) startDisplaying = true;
   //      if ((time != null) && (time.getDoubleValue() >= 1.6632+1e-7))
   //      {
   //         stopDisplaying = true; //1.6705
   //         writer.close();
   //      }
   //
   //      if (startDisplaying & !stopDisplaying)
   //      {
   //         if ((Math.abs(time.getDoubleValue() - 1.6632) < 0.00005) && (this.getName().equals("o_tau_rh_roll")))
   //         {
   //            System.out.println("time = " + time.getDoubleValue());
   //            System.out.println(this.getName() + " is getting set to " + value);
   //         }
   //
   //         if (!this.name.contains("DurationMilli"))
   //         {
   //            writer.println(time.getDoubleValue() + ": " + this.name + " = " + value);
   //            //       System.out.println(time.getDoubleValue() + ": " + this.name + " = " + value);
   //         }
   //      }
   //
   //   }
}
