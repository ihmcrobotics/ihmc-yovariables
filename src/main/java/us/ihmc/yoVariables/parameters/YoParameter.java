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
import java.util.NoSuchElementException;

import us.ihmc.yoVariables.listener.YoParameterChangedListener;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoTools;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Base class for parameters. Parameters cannot be changed from code and are only changed by the
 * user/operator. Available implementations
 * <ul>
 * <li>BooleanParameter
 * <li>DoubleParameter
 * <li>EnumParameter
 * <li>IntegerParameter
 * <li>LongParameter
 * </ul>
 *
 * @author Jesper Smith
 * @param <T>
 */
public abstract class YoParameter
{
   private final String name;
   private final String description;
   protected ParameterLoadStatus loadStatus = ParameterLoadStatus.UNLOADED;
   private YoParameterChangedListenerHolder changedListenerHolder;

   /**
    * @return the name of this parameter
    */
   public String getName()
   {
      return name;
   }

   /**
    * User readable description that describes the purpose of this parameter The description is only
    * used as a guideline to the user.
    *
    * @return
    */
   public String getDescription()
   {
      return description;
   }

   /**
    * @return the namespace of this parameter
    */
   public NameSpace getNameSpace()
   {
      return getVariable().getNameSpace();
   }

   /**
    * Attaches an object implementing {@link YoParameterChangedListener} to this parameter's list of
    * listeners.
    * <p>
    * Instantiates a new list of listeners if it is currently empty.
    * </p>
    *
    * @param YoParameterChangedListener ParameterChangedListener to attach
    */
   public void addParameterChangedListener(YoParameterChangedListener parameterChangedListener)
   {
      if (changedListenerHolder == null)
      {
         changedListenerHolder = new YoParameterChangedListenerHolder();
         getVariable().addListener(changedListenerHolder);
      }

      this.changedListenerHolder.add(parameterChangedListener);
   }

   /**
    * Clears this parameter's list of {@link YoParameterChangedListener}s.
    * <p>
    * If the list is null, does nothing.
    * </p>
    */
   public void removeAllParameterChangedListeners()
   {
      if (this.changedListenerHolder != null)
      {
         this.changedListenerHolder.clear();
      }
   }

   /**
    * Returns this parameter's list of {@link YoParameterChangedListener}s.
    *
    * @return List of change listeners, null if empty
    */
   public List<YoParameterChangedListener> getParameterChangedListeners()
   {
      if (this.changedListenerHolder == null)
      {
         return null;
      }
      else
      {
         return this.changedListenerHolder.getParameterChangedListeners();
      }
   }

   /**
    * Removes a {@link YoParameterChangedListener} from this parameter's list of listeners.
    *
    * @param YoParameterChangedListener ParameterChangedListener to remove
    */
   public void removeParameterChangedListener(YoParameterChangedListener parameterChangedListener)
   {
      boolean success;

      if (changedListenerHolder == null)
         success = false;
      else
         success = this.changedListenerHolder.remove(parameterChangedListener);

      if (!success)
         throw new NoSuchElementException("Listener not found");
   }

   /**
    * Get the value of this parameter as a string. The value depends on the type, numeric types will
    * return a numeric representation while enum's will return the enum value string.
    *
    * @return the value as string
    */
   public abstract String getValueAsString();

   YoParameter(String name, String description)
   {
      YoTools.checkForIllegalCharacters(name);
      this.name = name;
      this.description = description;
   }

   abstract YoVariable getVariable();

   abstract void setToString(String valueString);

   abstract void setToDefault();

   void setSuggestedRange(double min, double max)
   {
      getVariable().setVariableBounds(min, max);
   }

   public double getManualScalingMin()
   {
      return getVariable().getLowerBound();
   }

   public double getManualScalingMax()
   {
      return getVariable().getUpperBound();
   }

   void checkLoaded()
   {
      if (loadStatus == ParameterLoadStatus.UNLOADED)
      {
         throw new RuntimeException("Cannot use parameter " + name + " before it's value is loaded.");
      }
   }

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
    * Check if this parameter has been loaded
    *
    * @return true if this parameter has been loaded and can be used
    */
   public boolean isLoaded()
   {
      return loadStatus != ParameterLoadStatus.UNLOADED;
   }

   /**
    * Helper class to delegate VariableChangedListeners to ParameterChangedListeners
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

      public boolean remove(YoParameterChangedListener listener)
      {
         return changedListeners.remove(listener);
      }

      public List<YoParameterChangedListener> getParameterChangedListeners()
      {
         return changedListeners;
      }

      public void clear()
      {
         changedListeners.clear();
      }

      public void add(YoParameterChangedListener listener)
      {
         changedListeners.add(listener);
      }
   }
}
