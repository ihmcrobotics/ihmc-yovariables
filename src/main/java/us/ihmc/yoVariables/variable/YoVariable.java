package us.ihmc.yoVariables.variable;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;

import us.ihmc.yoVariables.listener.VariableChangedListener;
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
 * stored in the DataBuffer which may be exported for later use.
 * </p>
 */
public abstract class YoVariable
{
   private List<VariableChangedListener> variableChangedListeners;
   private double manualMinScaling = 0.0, manualMaxScaling = 1.0;

   private final String name;
   private final String description;
   private final YoVariableType type;
   private YoRegistry registry;

   /**
    * Create a new YoVariable. This is called by extensions of YoVariable, and require a
    * {@link YoVariableType}, name, description, and {@link YoRegistry} to register itself to.
    *
    * @param type        YoVariableType for this YoVariable to implement
    * @param name        String that uniquely identifies this YoVariable
    * @param description String that describes this YoVariable's purpose
    * @param registry    YoVariableRegistry for this YoVariable to register itself to after
    *                    initialization
    */
   public YoVariable(YoVariableType type, String name, String description, YoRegistry registry)
   {
      YoTools.checkForIllegalCharacters(name);

      this.type = type;
      this.name = name;
      this.description = description;
      this.variableChangedListeners = null;

      if (registry != null)
         registry.addVariable(this);
   }

   public void setRegistry(YoRegistry registry)
   {
      this.registry = registry;
   }

   /**
    * Retrieves the {@link YoRegistry} this variable belongs to.
    *
    * @return YoVariableRegistry this variable is registered in
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
      return this.name;
   }

   /**
    * Retrieve the description of this variable, "" if not specified.
    *
    * @return the description of this variable
    */
   public String getDescription()
   {
      return this.description;
   }

   /**
    * Set the min and max scaling values for graphing purposes. By default graphs are created using
    * manual scaling based on these values where min = 0.0 and max = 1.0.
    *
    * @param minScaling double representing the min scale value
    * @param maxScaling double representing the max scale value
    */
   public void setManualScalingMinMax(double minScaling, double maxScaling)
   {
      this.manualMinScaling = minScaling;
      this.manualMaxScaling = maxScaling;
   }

   /**
    * Retrieve the current minimum value for manual scaling.
    *
    * @return double min value
    */
   public double getManualScalingMin()
   {
      return manualMinScaling;
   }

   /**
    * Retrieve the current maximum value for manual scaling.
    *
    * @return double max value
    */
   public double getManualScalingMax()
   {
      return manualMaxScaling;
   }

   /**
    * Retrieves this variable's {@link YoVariableType}.
    *
    * @return YoVariableType of this variable
    */
   public final YoVariableType getYoVariableType()
   {
      return type;
   }

   /**
    * Retrieves this variable's full name and namespace, if applicable.
    *
    * @return String fully qualified name
    */
   public String getFullNameWithNameSpace()
   {
      if (registry == null)
         return this.name;
      if (registry.getNameSpace() == null)
         return this.name;

      return registry.getNameSpace() + "." + this.name;
   }

   /**
    * Retrieves this variable's namespace alone.
    *
    * @return String namespace for this variable
    */
   public NameSpace getNameSpace()
   {
      return registry.getNameSpace();
   }

   /**
    * Attaches an object implementing {@link VariableChangedListener} to this variable's list of
    * listeners.
    * <p>
    * Instantiates a new empty list of listeners if it is currently null.
    * </p>
    *
    * @param variableChangedListener VariableChangedListener to attach
    */
   public void addVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      if (variableChangedListeners == null)
      {
         variableChangedListeners = new ArrayList<>();
      }

      this.variableChangedListeners.add(variableChangedListener);
   }

   /**
    * Clears this variable's list of {@link VariableChangedListener}s.
    * <p>
    * If the list is null, does nothing.
    * </p>
    */
   public void removeAllVariableChangedListeners()
   {
      if (this.variableChangedListeners != null)
      {
         this.variableChangedListeners.clear();
      }
   }

   /**
    * Returns this variable's list of {@link VariableChangedListener}s.
    *
    * @return
    */
   public List<VariableChangedListener> getVariableChangedListeners()
   {
      return variableChangedListeners;
   }

   /**
    * Removes a {@link VariableChangedListener} from this variable's list of listeners.
    *
    * @param variableChangedListener VariableChangedListener to remove
    */
   public void removeVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      boolean success;

      if (variableChangedListeners == null)
         success = false;
      else
         success = this.variableChangedListeners.remove(variableChangedListener);

      if (!success)
         throw new NoSuchElementException("Listener not found");
   }

   /**
    * Calls {@link VariableChangedListener#notifyOfVariableChange(YoVariable)} with this variable for
    * every {@link VariableChangedListener} attached to this variable.
    */
   public void notifyVariableChangedListeners()
   {
      if (variableChangedListeners != null)
      {
         for (int i = 0; i < variableChangedListeners.size(); i++)
         {
            variableChangedListeners.get(i).notifyOfVariableChange(this);
         }
      }
   }

   /**
    * Retrieves this variable's value interpreted as a double.
    * <p>
    * Abstract; implemented by each extension of YoVariable to return different interpretations.
    * </p>
    *
    * @return double value of variable interpreted as Double
    */
   public abstract double getValueAsDouble();

   /**
    * Calls {@link #setValueFromDouble(double, boolean)} with (value, true).
    *
    * @param value double to set this variable's value to
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
    * Will call {@link #notifyVariableChangedListeners()} if notifyListeners is true.
    * </p>
    *
    * @param value           double to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call
    *                        {@link #notifyVariableChangedListeners()}
    */
   public abstract void setValueFromDouble(double value, boolean notifyListeners);

   /**
    * Retrieves this variable's value interpreted as a long.
    * <p>
    * Abstract; implemented by each extension of YoVariable to result in different interpretation.
    * </p>
    *
    * @return long formatted value of this variable
    */
   public abstract long getValueAsLongBits();

   /**
    * Calls {@link #setValueFromLongBits(long, boolean)} with (value, true).
    *
    * @param value long value to set this variable to
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
    * @param value           long to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call
    *                        {@link #notifyVariableChangedListeners()}
    */
   public abstract void setValueFromLongBits(long value, boolean notifyListeners);

   /**
    * Creates a copy of this variable in the given {@link YoRegistry}
    * <p>
    * Abstract; implemented by each extension of YoVariable to perform action with proper typing.
    * </p>
    *
    * @param newRegistry YoVariableRegistry to duplicate this variable to
    * @return the newly created variable from the given newRegistry
    */
   public abstract YoVariable duplicate(YoRegistry newRegistry);

   /**
    * Assesses if this variable's value is equivalent to zero.
    * <p>
    * Abstract; implemented by each extension of YoVariable to result in different interpretation.
    * </p>
    *
    * @return boolean if variable's value is zero, per extension's interpretation
    */
   public abstract boolean isZero();

   /**
    * Flag to notify if this variable should be treated as a parameter
    *
    * @return true if this a parameter
    */
   public boolean isParameter()
   {
      return false;
   }

   /**
    * If isParamater() is true, this function returns the corresponding parameter object
    *
    * @return the parameter object if isParameter() is true, null otherwise
    */
   public YoParameter<?> getParameter()
   {
      return null;
   }
}
