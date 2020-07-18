package us.ihmc.yoVariables.variable;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoTools;

/**
 * {@code YoVariable}s provide the base for a framework that allows storing, manipulating, logging,
 * and visualizing data.
 * <p>
 * Each {@code YoVariable} can be retrieved through its parent {@code YoRegistry}. When creating a
 * module processing data such as a controller, all of its {@code YoVariable}s can be retrieved when
 * registered to a given {@code YoRegistry}, allowing then for instance to collect all the variables
 * and to publish them on the server such as a client can read the variables.
 * </p>
 * 
 * @see YoBoolean
 * @see YoDouble
 * @see YoInteger
 * @see YoLong
 * @see YoEnum
 * @see YoParameter
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
    * @param type        used to represent the implementation of this variable.
    * @param name        the name for this variable that can be used to retrieve it from a
    *                    {@link YoRegistry}.
    * @param description description of this variable's purpose.
    * @param registry    initial parent registry for this variable.
    */
   public YoVariable(YoVariableType type, String name, String description, YoRegistry registry)
   {
      YoTools.checkForIllegalCharacters(name);

      this.type = type;
      this.name = name;
      this.description = description;
      setRegistry(registry);
   }

   /**
    * Sets the registry for this variable.
    * 
    * @param registry the new registry in which this variable is registry. If {@code null}, the
    *                 variable is detached from its previous registry.
    */
   public void setRegistry(YoRegistry registry)
   {
      if (registry == this.registry)
         return;

      if (this.registry != null)
         this.registry.removeVariable(this);
      if (registry != null)
         registry.addVariable(this);

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
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    */
   public final boolean setValueFromDouble(double value)
   {
      return setValueFromDouble(value, true);
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
    * @param notifyListeners whether or not to notify this variable's listeners in the case this
    *                        variable's value changed.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    */
   public abstract boolean setValueFromDouble(double value, boolean notifyListeners);

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
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    */
   public final boolean setValueFromLongBits(long value)
   {
      return setValueFromLongBits(value, true);
   }

   /**
    * Sets this variable's value using the given long.
    * <p>
    * Abstract; implemented by each extension of YoVariable to result in different action taken.
    * </p>
    *
    * @param value           long to set this variable's value to.
    * @param notifyListeners whether or not to notify this variable's listeners in the case this
    *                        variable's value changed.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    */
   public abstract boolean setValueFromLongBits(long value, boolean notifyListeners);

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
    * @param notifyListeners whether or not to notify this variable's listeners in the case this
    *                        variable's value changed.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
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
    * @param format the format to use for a double value. Can be {@code null}.
    * @return the string representation of this variable's current value.
    */
   public abstract String getValueAsString(String format);

   /**
    * Tries to parse the given string and to set this variable's value.
    * <p>
    * This variable's listeners will be notified if this variable's value is changed.
    * </p>
    * 
    * @param valueAsString the string to parse.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    */
   public boolean parseValue(String valueAsString)
   {
      return parseValue(valueAsString, true);
   }

   /**
    * Tries to parse the given string and to set this variable's value.
    * <p>
    * If {@code notifyListeners} is {@code true}, this variable's listeners will be notified if the
    * value parsed changes the the current value for this variable.
    * </p>
    * 
    * @param valueAsString   the string to parse.
    * @param notifyListeners whether to notify this variable's listeners if this operation results in
    *                        changing this variable's current value.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    */
   public abstract boolean parseValue(String valueAsString, boolean notifyListeners);

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
   public YoParameter getParameter()
   {
      return null;
   }

   /**
    * Clones this variable and registers the clone to the given registry.
    *
    * @param newRegistry the registry in which the clone is registered.
    * @return the cloned variable.
    */
   public abstract YoVariable duplicate(YoRegistry newRegistry);

   /**
    * Returns {@code String} representation of this variable.
    *
    * @return {@code String} representing this variable and its current value.
    */
   @Override
   public abstract String toString();
}
