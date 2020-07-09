package us.ihmc.yoVariables.variable;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import us.ihmc.yoVariables.registry.YoTools;

public class YoVariableList implements java.io.Serializable, java.lang.Comparable<YoVariableList>
{
   private static final long serialVersionUID = -393664925453518934L;
   private final ArrayList<ChangeListener> listeners;
   private final String name;
   private final List<YoVariable> variables;
   private final Map<String, List<YoVariable>> variablesMappedByName;

   public YoVariableList(String name)
   {
      this.name = name;
      variables = new ArrayList<>();
      variablesMappedByName = new LinkedHashMap<>();

      listeners = new ArrayList<>();
   }

   public String getName()
   {
      return name;
   }

   @Override
   public String toString()
   {
      StringBuffer retBuffer = new StringBuffer();

      for (int i = 0; i < variables.size(); i++)
      {
         retBuffer.append(variables.get(i).toString());
         retBuffer.append("\n");
      }

      return retBuffer.toString();
   }

   public boolean isEmpty()
   {
      return variables.size() == 0;
   }

   public void addVariable(YoVariable variable)
   {
      String variableName = variable.getName();
      List<YoVariable> arrayList = variablesMappedByName.get(variableName);
      if (arrayList == null)
      {
         arrayList = new ArrayList<>(1);
         variablesMappedByName.put(variable.getName(), arrayList);
      }

      if (!arrayList.contains(variable))
      {
         variables.add(variable);
         arrayList.add(variable);

         notifyListeners();
      }
   }

   /**
    * Tell all listeners that a change in the variable count occurred. At time of writing only
    * VarPanelsHolders are listeners for this.
    */
   private void notifyListeners()
   {
      for (ChangeListener listener : listeners)
      {
         listener.stateChanged(new ChangeEvent(this));
      }
   }

   public void addVariables(YoVariableList controlVars)
   {
      List<YoVariable> variables = controlVars.getVariables();

      this.addVariables(variables);
   }

   public void addVariables(List<YoVariable> list)
   {
      for (YoVariable variable : list)
      {
         addVariable(variable);
      }
   }

   public void addVariables(YoVariable[] variables)
   {
      for (int i = 0; i < variables.length; i++)
      {
         addVariable(variables[i]);
      }
   }

   public void removeVariable(YoVariable variable)
   {
      String variableName = variable.getName();

      if (variablesMappedByName.containsKey(variableName))
      {
         List<YoVariable> arrayList = variablesMappedByName.get(variableName);
         arrayList.remove(variable);
         variables.remove(variable);
      }
   }

   public void removeAllVariables()
   {
      variables.clear();
      variablesMappedByName.clear();

      notifyListeners();
   }

   public boolean containsVariable(YoVariable variable)
   {
      String variableName = variable.getName();
      List<YoVariable> arrayList = variablesMappedByName.get(variableName);

      if (arrayList != null && arrayList.contains(variable))
         return true;

      return false;
   }

   public List<YoVariable> getVariables()
   {
      List<YoVariable> ret = new ArrayList<>();
      ret.addAll(variables);

      return ret;
   }

   public int size()
   {
      return variables.size();
   }

   public synchronized YoVariable getVariable(int index)
   {
      if (index <= variables.size() - 1 && index >= 0)
      {
         return variables.get(index);
      }

      return null;
   }

   public synchronized YoVariable getVariable(String name)
   {
      List<YoVariable> arrayList;
      int lastDotIndex = name.lastIndexOf(".");

      if (lastDotIndex == -1)
      {
         arrayList = variablesMappedByName.get(name);
      }
      else
      {
         String endOfName = name.substring(lastDotIndex + 1);
         arrayList = variablesMappedByName.get(endOfName);
      }

      if (arrayList == null)
         return null;

      for (int i = 0; i < arrayList.size(); i++)
      {
         YoVariable variable = arrayList.get(i);

         if (YoTools.matchFullNameEndsWithCaseInsensitive(variable, name))
         {
            return variable;
         }
      }

      return null;
   }

   public synchronized String[] getVariableNames()
   {
      String[] ret = new String[variables.size()];

      for (int i = 0; i < variables.size(); i++)
      {
         ret[i] = variables.get(i).getName();
      }

      return ret;
   }

   public synchronized boolean hasVariableWithName(String name)
   {
      if (getVariable(name) != null)
      {
         return true;
      }

      return false;
   }

   // TODO: duplicated in YoVariableRegistry
   public synchronized List<YoVariable> getMatchingVariables(String[] names, String[] regularExpressions)
   {
      List<YoVariable> ret = new ArrayList<>();

      if (names != null)
      {
         for (int i = 0; i < names.length; i++)
         {
            String name = names[i];
            YoVariable var = getVariable(name);

            if (var != null)
            {
               ret.add(var);
            }
         }
      }

      if (regularExpressions != null)
      {
         for (int i = 0; i < regularExpressions.length; i++)
         {
            Pattern pattern = Pattern.compile(regularExpressions[i]);

            for (int j = 0; j < variables.size(); j++)
            {
               YoVariable var = variables.get(j);
               Matcher matcher = pattern.matcher(var.getName());

               if (matcher.matches())
               {
                  ret.add(var);
               }
            }
         }
      }

      return ret;
   }

   public YoVariable[] getAllVariables()
   {
      YoVariable[] ret = new YoVariable[variables.size()];

      variables.toArray(ret);

      return ret;
   }

   public void addChangeListener(ChangeListener listener)
   {
      listeners.add(listener);
   }

   /**
    * // * Compares this VarList to the specified object returning > = < as 1 0 -1 respectively. // *
    * Reference object must be another VarList otherwise a runtime exception will be thrown. / //
    * * @param other Object to which this will be compared // * @return indicates > = < as 1 0 -1
    * respectively /
    */
   @Override
   public int compareTo(YoVariableList other)
   {
      return (int) Math.signum(getName().compareToIgnoreCase(other.getName()));

   }

   public synchronized int getIndexOfVariable(YoVariable variable)
   {
      return variables.indexOf(variable);
   }
}
