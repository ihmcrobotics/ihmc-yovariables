package us.ihmc.yoVariables.variable;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoTools;

/**
 * Title: Simulation Construction Set
 * <p>
 * Description: Package for Simulating Dynamic Robots and Mechanisms
 * <p>
 * YoVariables provide a simple, convenient mechanism for storing and manipulating robot data. While
 * each essentially contains a double value YoVariables are designed for integration into the SCS
 * GUI. Once registered, a variable will automatically become available to the GUI for graphing,
 * modification and other data manipulation. Historical values of all registered YoVariables are
 * stored in the {@link DataBuffer} which may be exported for later use.
 * </p>
 */
public abstract class YoVariable
{
   private final String name;
   private final String description;
   private final YoVariableType type;
   private YoRegistry registry;
   private NameSpace fullName;

   private List<YoVariableChangedListener> changedListeners;
   private double lowerBound = 0.0;
   private double upperBound = 1.0;

   /**
    * Create a new YoVariable. This is called by extensions of YoVariable, and require a
    * {@link YoVariableType}, name, description, and {@link YoRegistry} to register itself to.
    *
    * @param type        YoVariableType for this YoVariable to implement
    * @param name        String that uniquely identifies this YoVariable
    * @param description String that describes this YoVariable's purpose
    * @param registry    YoRegistry for this YoVariable to register itself to after initialization
    */
   public YoVariable(YoVariableType type, String name, String description, YoRegistry registry)
   {
      YoTools.checkForIllegalCharacters(name);

      this.type = type;
      this.name = name;
      this.description = description;

      if (registry != null)
         registry.addVariable(this);
   }

   public void setRegistry(YoRegistry registry)
   {
      this.registry = registry;
      fullName = null; // Force to reset the fullName so it is updated on next query.
   }

   /**
    * Retrieves the {@link YoRegistry} this variable belongs to.
    *
    * @return YoRegistry this variable is registered in
    */
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   /**
    * Retrieves the name of this YoVariable.
    *
    * @return the full name of this variable
    */
   public String getName()
   {
      return name;
   }

   /**
    * Retrieve the description of this variable, "" if not specified.
    *
    * @return the description of this variable
    */
   public String getDescription()
   {
      return description;
   }

   /**
    * Sets the bounds for this variable's range of values.
    * <p>
    * Variable bounds are typically used when interacting with the variable via a GUI. For instance,
    * the variable bounds can be used to set the bounds of control slider or set the range when
    * plotting this variable.
    * </p>
    * <p>
    * Note that nothing in the implementation of a {@code YoVariable} enforces the value to remain
    * within its current bounds, it is only for facilitating the definition of bounds and tracking to
    * the bounds' owner.
    * </p>
    *
    * @param lowerBound double value representing the lower bound for this variable.
    * @param upperBound double value representing the upper bound for this variable.
    */
   public void setVariableBounds(double lowerBound, double upperBound)
   {
      this.lowerBound = lowerBound;
      this.upperBound = upperBound;
   }

   /**
    * Returns the current double value representing the lower bound for this variable's value range.
    * <p>
    * Variable bounds are typically used when interacting with the variable via a GUI. For instance,
    * the variable bounds can be used to set the bounds of control slider or set the range when
    * plotting this variable.
    * </p>
    * <p>
    * Note that nothing in the implementation of a {@code YoVariable} enforces the value to remain
    * within its current bounds, it is only for facilitating the definition of bounds and tracking to
    * the bounds' owner.
    * </p>
    *
    * @return minimum value as double for this variable.
    */
   public double getLowerBound()
   {
      return lowerBound;
   }

   /**
    * Returns the current double value representing the upper bound for this variable's value range.
    * <p>
    * Variable bounds are typically used when interacting with the variable via a GUI. For instance,
    * the variable bounds can be used to set the bounds of control slider or set the range when
    * plotting this variable.
    * </p>
    * <p>
    * Note that nothing in the implementation of a {@code YoVariable} enforces the value to remain
    * within its current bounds, it is only for facilitating the definition of bounds and tracking to
    * the bounds' owner.
    * </p>
    *
    * @return maximum value as double for this variable.
    */
   public double getUpperBound()
   {
      return upperBound;
   }

   /**
    * Retrieves this variable's {@link YoVariableType}.
    *
    * @return YoVariableType of this variable
    */
   public final YoVariableType getType()
   {
      return type;
   }

   /**
    * Retrieves this variable's full name, i.e. this variable prepended with its parent's namespace if
    * applicable, and returns the full name as a namespace.
    *
    * @return this variable's full name as a namespace.
    */
   public NameSpace getFullName()
   {
      if (fullName == null)
      {
         if (registry == null)
            fullName = new NameSpace(name);
         else
            fullName = registry.getNameSpace().append(name);
      }

      return fullName;
   }

   /**
    * Retrieves this variable's full name, i.e. this variable prepended with its parent's namespace if
    * applicable.
    *
    * @return this variable's full name.
    */
   public String getFullNameString()
   {
      return getFullName().getName();
   }

   /**
    * Returns the namespace of the registry in which this variable is currently registered to, or
    * {@code null} if this variable is not registered to any registry.
    *
    * @return this variable's namespace.
    */
   public NameSpace getNameSpace()
   {
      return registry == null ? null : registry.getNameSpace();
   }

   /**
    * Adds a listener to this variable.
    *
    * @param listener the listener for listening to changes done to this variable.
    */
   public void addListener(YoVariableChangedListener listener)
   {
      if (changedListeners == null)
         changedListeners = new ArrayList<>();

      changedListeners.add(listener);
   }

   /**
    * Removes all listeners previously added to this variable.
    */
   public void removeListeners()
   {
      changedListeners = null;
   }

   /**
    * Returns this variable's list of {@link YoVariableChangedListener}s.
    *
    * @return the listeners previously added to this variable, or {@code null} if this variable has no
    *         listener.
    */
   public List<YoVariableChangedListener> getListeners()
   {
      return changedListeners;
   }

   /**
    * Tries to remove a listener from this variable. If the listener could not be found and removed,
    * nothing happens.
    *
    * @param listener the listener to remove.
    * @return {@code true} if the listener was removed, {@code false} if the listener was not found and
    *         nothing happened.
    */
   public boolean removeListener(YoVariableChangedListener listener)
   {
      if (changedListeners == null)
         return false;
      else
         return changedListeners.remove(listener);
   }

   /**
    * Triggers a notification to all the listeners currently attached to this variable.
    */
   public void notifyListeners()
   {
      if (changedListeners != null)
      {
         for (int i = 0; i < changedListeners.size(); i++)
         {
            changedListeners.get(i).changed(this);
         }
      }
   }

   /**
    * Retrieves this variable's value interpreted as a double.
    * <p>
    * Abstract; implemented by each extension of YoVariable to return different interpretations.
    * </p>
    *
    * @return current value of this variable interpreted as double.
    */
   public abstract double getValueAsDouble();

   /**
    * Calls {@link #setValueFromDouble(double, boolean)} with (value, true).
    *
    * @param value double to set this variable's value to.
    */
   public final void setValueFromDouble(double value)
   {
      setValueFromDouble(value, true);
   }

   /**
    * Sets this variable's value using the given double.
    * <p>
    * Abstract; implemented by each extension of YoVariable to result in different interpretation.
    * </p>
    * <p>
    * Will call {@link #notifyListeners()} if notifyListeners is true.
    * </p>
    *
    * @param value           double to set this variable's value to.
    * @param notifyListeners boolean determining whether or not to call {@link #notifyListeners()}.
    */
   public abstract void setValueFromDouble(double value, boolean notifyListeners);

   /**
    * Retrieves this variable's value interpreted as a long.
    * <p>
    * Abstract; implemented by each extension of YoVariable to result in different interpretation.
    * </p>
    *
    * @return long formatted value of this variable.
    */
   public abstract long getValueAsLongBits();

   /**
    * Calls {@link #setValueFromLongBits(long, boolean)} with (value, true).
    *
    * @param value long value to set this variable to.
    */
   public final void setValueFromLongBits(long value)
   {
      setValueFromLongBits(value, true);
   }

   /**
    * Sets this variable's value using the given long.
    * <p>
    * Abstract; implemented by each extension of YoVariable to result in different action taken.
    * </p>
    *
    * @param value           long to set this variable's value to.
    * @param notifyListeners boolean determining whether or not to call {@link #notifyListeners()}.
    */
   public abstract void setValueFromLongBits(long value, boolean notifyListeners);

   /**
    * Creates a copy of this variable in the given {@link YoRegistry}.
    * <p>
    * Abstract; implemented by each extension of YoVariable to perform action with proper typing.
    * </p>
    *
    * @param newRegistry YoRegistry to duplicate this variable to.
    * @return the newly created variable from the given newRegistry.
    */
   public abstract YoVariable duplicate(YoRegistry newRegistry);

   /**
    * Sets this variable's value to the given value.
    * <p>
    * Abstract; implemented by each extension of {@code YoVariable} to perform action with proper
    * typing.
    * </p>
    * <p>
    * Note that {@code other} is casted to the final implementation of this {@code YoVariable}.
    * </p>
    *
    * @param other           the other variable used to set the value of {@code this}. It is assumed to
    *                        be of the same type as {@code this}.
    * @param notifyListeners boolean determining whether or not to call {@link #notifyListeners()}
    * @return true if value changed
    */
   public abstract boolean setValue(YoVariable other, boolean notifyListeners);

   /**
    * Returns the value of this variable as a string.
    * <p>
    * The returned string is expected to not be following any specific formatting, for instance for a
    * {@code YoDouble} this relies on {@link Double#toString(double)}.
    * </p>
    * 
    * @return the string representation of this variable's current value.
    */
   public String getValueAsString()
   {
      return getValueAsString(null);
   }

   /**
    * Returns the value of this variable as a string using the given format if this is a
    * {@link YoDouble}.
    * 
    * @param format the format to use for a double value.
    * @return the string representation of this variable's current value.
    */
   public abstract String getValueAsString(String format);

   /**
    * Assesses if this variable's value is equivalent to zero.
    * <p>
    * Abstract; implemented by each extension of YoVariable to result in different interpretation.
    * </p>
    *
    * @return boolean if variable's value is zero, per extension's interpretation.
    */
   public abstract boolean isZero();

   /**
    * Flag to notify if this variable should be treated as a parameter.
    *
    * @return true if this a parameter.
    */
   public boolean isParameter()
   {
      return false;
   }

   /**
    * If {@link #isParameter()} is true, this function returns the corresponding parameter object.
    *
    * @return the parameter object if {@link #isParameter()} is true, {@code null} otherwise.
    */
   public YoParameter<?> getParameter()
   {
      return null;
   }
}
