/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package us.ihmc.yoVariables.parameters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.yoVariables.exceptions.IllegalOperationException;
import us.ihmc.yoVariables.listener.YoParameterChangedListener;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Base class for parameters.
 * <p>
 * A parameter can be seen as a read-only {@link YoVariable}. The intention for parameters is the
 * guarantee of not being modified by the algorithm/controller/module that uses the parameter's
 * value for computation. This allows to guarantee the user of a graphical user interface, such as
 * Simulation Construction Set, that he/she is the only entity able to modify the parameter's value.
 * </p>
 * <p>
 * A parameter can be initialized with an given value, note that process requires manual
 * intervention. Parameters can be initialized only once and have to be initialized before they can
 * be used. The initialization can be performed by using a parameter reader, i.e. implementation of
 * {@link AbstractParameterReader}. Two default implementations are provided in this package:
 * {@link DefaultParameterReader} which initializes the parameters to the value that was passed at
 * construction of each parameter, {@link XmlParameterReader} that parses the initial values from a
 * given XML file.
 * </p>
 *
 * @author Jesper Smith
 * @see YoVariable
 * @see BooleanParameter
 * @see DoubleParameter
 * @see EnumParameter
 * @see IntegerParameter
 * @see LongParameter
 */
public abstract class YoParameter
{
   /** Field used to keep track of the initialization state of this parameter. */
   protected ParameterLoadStatus loadStatus = ParameterLoadStatus.UNLOADED;
   /** Manager of the {@code YoParameterChangedListener}s added to this variable. */
   private YoParameterChangedListenerHolder changedListenerHolder;

   /**
    * Retrieves the name of this parameter.
    *
    * @return the name of this parameter.
    */
   public String getName()
   {
      return getVariable().getName();
   }

   /**
    * Retrieve the description of this parameter purpose, "" if not specified.
    *
    * @return the description of this parameter.
    */
   public String getDescription()
   {
      return getVariable().getDescription();
   }

   /**
    * Retrieves this parameter's full name, i.e. this parameter name prepended with its parent's
    * namespace if applicable, and returns the full name as a namespace.
    *
    * @return this parameter's full name as a namespace.
    */
   public YoNamespace getFullName()
   {
      return getVariable().getFullName();
   }

   /**
    * Retrieves this parameter's full name, i.e. this parameter prepended with its parent's namespace
    * if applicable.
    *
    * @return this parameter's full name.
    */
   public String getFullNameString()
   {
      return getVariable().getFullNameString();
   }

   /**
    * Returns the namespace of the registry in which this parameter is currently registered to, or
    * {@code null} if this parameter is not registered to any registry.
    *
    * @return this parameter's namespace.
    */
   public YoNamespace getNamespace()
   {
      return getVariable().getNamespace();
   }

   /**
    * Sets the bounds for this parameter's range of values.
    * <p>
    * Parameter bounds are typically used when interacting with the parameter via a GUI. For instance,
    * the parameter bounds can be used to set the bounds of control slider or set the range when
    * plotting this variable.
    * </p>
    * <p>
    * Note that nothing in the implementation of a {@code YoParameter} enforces the value to remain
    * within its current bounds, it is only for facilitating the definition of bounds and tracking to
    * the bounds' owner.
    * </p>
    *
    * @param lowerBound double value representing the lower bound for this parameter. Not enforced.
    * @param upperBound double value representing the upper bound for this parameter. Not enforced.
    */
   void setParameterBounds(double lowerBound, double upperBound)
   {
      getVariable().setVariableBounds(lowerBound, upperBound);
   }

   /**
    * Returns the current double value representing the lower bound for this parameter's value range.
    * <p>
    * Parameter bounds are typically used when interacting with the parameter via a GUI. For instance,
    * the parameter bounds can be used to set the bounds of control slider or set the range when
    * plotting this parameter.
    * </p>
    * <p>
    * Note that nothing in the implementation of a {@code YoParameter} enforces the value to remain
    * within its current bounds, it is only for facilitating the definition of bounds and tracking to
    * the bounds' owner.
    * </p>
    *
    * @return minimum value as double for this parameter.
    */
   public double getLowerBound()
   {
      return getVariable().getLowerBound();
   }

   /**
    * Returns the current double value representing the upper bound for this parameter's value range.
    * <p>
    * Parameter bounds are typically used when interacting with the parameter via a GUI. For instance,
    * the parameter bounds can be used to set the bounds of control slider or set the range when
    * plotting this parameter.
    * </p>
    * <p>
    * Note that nothing in the implementation of a {@code YoParameter} enforces the value to remain
    * within its current bounds, it is only for facilitating the definition of bounds and tracking to
    * the bounds' owner.
    * </p>
    *
    * @return maximum value as double for this parameter.
    */
   public double getUpperBound()
   {
      return getVariable().getUpperBound();
   }

   /**
    * Adds a listener to this parameter.
    *
    * @param listener the listener for listening to changes done to this parameter.
    */
   public void addListener(YoParameterChangedListener listener)
   {
      if (changedListenerHolder == null)
      {
         changedListenerHolder = new YoParameterChangedListenerHolder();
         getVariable().addListener(changedListenerHolder);
      }

      changedListenerHolder.addListener(listener);
   }

   /**
    * Removes all listeners previously added to this parameter.
    */
   public void removeListeners()
   {
      if (changedListenerHolder != null)
         changedListenerHolder.removeListeners();
   }

   /**
    * Returns this parameter's list of {@link YoParameterChangedListener}s.
    *
    * @return the listeners previously added to this parameter, or {@code null} if this parameter has
    *         no listener.
    */
   public List<YoParameterChangedListener> getListeners()
   {
      if (changedListenerHolder == null)
         return null;
      else
         return changedListenerHolder.getListeners();
   }

   /**
    * Tries to remove a listener from this parameter. If the listener could not be found and removed,
    * nothing happens.
    *
    * @param listener the listener to remove.
    * @return {@code true} if the listener was removed, {@code false} if the listener was not found and
    *         nothing happened.
    */
   public boolean removeListener(YoParameterChangedListener listener)
   {
      if (changedListenerHolder == null)
         return false;
      else
         return changedListenerHolder.removeListener(listener);
   }

   /**
    * Returns the value of this parameter as a {@code String}.
    * <p>
    * The returned {@code String} depends on the type, numeric types will return a numeric
    * representation while enum's will return the enum value string.
    * </p>
    * 
    * @return the string representation of this parameter's current value.
    */
   public final String getValueAsString()
   {
      checkLoaded();
      return getVariable().getValueAsString();
   }

   /**
    * Returns the variable backing this parameter.
    * 
    * @return the internal variable.
    */
   abstract YoVariable getVariable();

   /**
    * Tries to parse the given string and to set this parameter's value.
    * <p>
    * This is typically used to initialize this parameter.
    * </p>
    * 
    * @param valueAsString the string to parse.
    * @throws IllegalArgumentException if the given string value could not be parsed.
    */
   final void setToString(String valueString)
   {
      getVariable().parseValue(valueString);
   }

   /**
    * Sets this parameter's value to the initial value that was provided at construction.
    * <p>
    * This is typically used to initialize this parameter.
    * </p>
    */
   abstract void setToDefault();

   void load(String valueString)
   {
      loadStatus = ParameterLoadStatus.LOADED;
      setToString(valueString);
   }

   void loadDefault()
   {
      loadStatus = ParameterLoadStatus.DEFAULT;
      setToDefault();
   }

   /**
    * Get the load status of this parameter. It will indicate whether the parameter was load from file,
    * is using its default value, or is still unloaded.
    *
    * @return the current {@link ParameterLoadStatus} of this parameter.
    */
   public ParameterLoadStatus getLoadStatus()
   {
      return loadStatus;
   }

   /**
    * Checks that this parameter has been initialized.
    * 
    * @throws IllegalOperationException if this parameter has not been initialized.
    */
   public void checkLoaded()
   {
      if (!isLoaded())
         throw new IllegalOperationException("The parameter " + getFullName() + " has not been loaded. This is required to enable its use.");
   }

   /**
    * Tests if this parameter has been loaded.
    *
    * @return {@code true} if this parameter has been loaded and can be used.
    */
   public boolean isLoaded()
   {
      return loadStatus != ParameterLoadStatus.UNLOADED;
   }

   /**
    * Helper class to delegate {@link YoVariableChangedListener}s to
    * {@link YoParameterChangedListener}s.
    *
    * @author Jesper Smith
    */
   private static class YoParameterChangedListenerHolder implements YoVariableChangedListener
   {
      private final List<YoParameterChangedListener> changedListeners = new ArrayList<>();

      @Override
      public void changed(YoVariable v)
      {
         for (int i = 0; i < changedListeners.size(); i++)
         {
            changedListeners.get(i).changed(v.getParameter());
         }
      }

      public void addListener(YoParameterChangedListener listener)
      {
         changedListeners.add(listener);
      }

      public void removeListeners()
      {
         changedListeners.clear();
      }

      public boolean removeListener(YoParameterChangedListener listener)
      {
         return changedListeners.remove(listener);
      }

      public List<YoParameterChangedListener> getListeners()
      {
         return changedListeners;
      }
   }
}
