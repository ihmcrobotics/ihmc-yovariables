package us.ihmc.yoVariables.registry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.yoVariables.exceptions.NameCollisionException;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * <p>
 * Description: An implementation of a YoVariableHolder.
 * </p>
 */
public class YoVariableList implements YoVariableHolder
{
   private final String name;
   private final List<YoVariable> variableList = new ArrayList<>();
   private final Map<String, List<YoVariable>> simpleNameToVariablesMap = new LinkedHashMap<>();

   public YoVariableList(String name)
   {
      this.name = name;
   }

   public YoVariableList(String name, List<? extends YoVariable> variables)
   {
      this.name = name;
      addVariables(variables);
   }

   public String getName()
   {
      return name;
   }

   public boolean isEmpty()
   {
      return variableList.isEmpty();
   }

   public int size()
   {
      return variableList.size();
   }

   public void clear()
   {
      variableList.clear();
      simpleNameToVariablesMap.clear();
   }

   public int indexOf(YoVariable variable)
   {
      return variableList.indexOf(variable);
   }

   public boolean contains(YoVariable variable)
   {
      List<YoVariable> homonyms = simpleNameToVariablesMap.get(variable.getName().toLowerCase());
      if (homonyms == null)
         return false;
      else
         return homonyms.contains(variable);
   }

   public YoVariable getVariable(int index)
   {
      return variableList.get(index);
   }

   @Override
   public List<YoVariable> getVariables()
   {
      return Collections.unmodifiableList(variableList);
   }

   /**
    * Adds the given YoVariables to this YoVariableHolder. If any Variable is not unique, throws a
    * RuntimeException.
    *
    * @param variables YoVariables to add to this YoVariableHolder
    */
   public void addVariables(List<? extends YoVariable> variables)
   {
      for (int i = 0; i < variables.size(); i++)
      {
         addVariable(variables.get(i));
      }
   }

   public void addVariables(YoVariableHolder yoVariableHolder)
   {
      addVariables(yoVariableHolder.getVariables());
   }

   /**
    * Adds the given YoVariable to this YoVariableHolder. If this Variable is not unique, throws a
    * RuntimeException.
    *
    * @param variable YoVariable to add to this YoVariableHolder
    */
   public void addVariable(YoVariable variable)
   {
      List<YoVariable> homonyms = simpleNameToVariablesMap.get(variable.getName().toLowerCase());

      if (homonyms == null)
      {
         homonyms = new ArrayList<>();
         simpleNameToVariablesMap.put(variable.getName().toLowerCase(), homonyms);
      }
      else
      {
         if (homonyms.contains(variable))
            return;

         // Make sure the variable is unique:
         for (int i = 0; i < homonyms.size(); i++)
         {
            if (homonyms.get(i).getNameSpace().equals(variable.getNameSpace()))
            {
               throw new NameCollisionException("Name collision with " + variable.getFullNameString());
            }
         }
      }

      homonyms.add(variable);
      variableList.add(variable);
   }

   public void removeVariable(YoVariable variable)
   {
      List<YoVariable> homonyms = simpleNameToVariablesMap.get(variable.getName().toLowerCase());
      if (homonyms != null)
      {
         homonyms.remove(variable);
         variableList.remove(variable);
      }
   }

   @Override
   public YoVariable findVariable(String nameSpaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      List<YoVariable> variableList = simpleNameToVariablesMap.get(name.toLowerCase());

      if (variableList == null || variableList.isEmpty())
         return null;

      if (nameSpaceEnding == null)
      {
         return variableList.get(0);
      }
      else
      {
         for (int i = 0; i < variableList.size(); i++)
         {
            YoVariable candidate = variableList.get(i);

            if (candidate.getNameSpace().endsWith(nameSpaceEnding, true))
               return candidate;
         }
      }

      return null;
   }

   @Override
   public boolean hasUniqueVariable(String nameSpaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      return countNumberOfVariables(nameSpaceEnding, name) == 1;
   }

   private int countNumberOfVariables(String parentNameSpace, String name)
   {
      List<YoVariable> variableList = simpleNameToVariablesMap.get(name.toLowerCase());

      if (variableList == null || variableList.isEmpty())
         return 0;

      if (parentNameSpace == null)
         return variableList.size();

      int count = 0;

      for (int i = 0; i < variableList.size(); i++)
      {
         if (variableList.get(i).getNameSpace().endsWith(parentNameSpace, true))
            count++;
      }
      return count;
   }

   @Override
   public List<YoVariable> findVariables(String nameSpaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      List<YoVariable> variableList = simpleNameToVariablesMap.get(name.toLowerCase());

      if (variableList == null || variableList.isEmpty())
         return Collections.emptyList();

      List<YoVariable> result = new ArrayList<>();

      if (nameSpaceEnding == null)
      {
         result.addAll(variableList);
      }
      else
      {
         for (int i = 0; i < variableList.size(); i++)
         {
            YoVariable candidate = variableList.get(i);

            if (candidate.getNameSpace().endsWith(nameSpaceEnding, true))
               result.add(candidate);
         }
      }

      return result;
   }

   @Override
   public List<YoVariable> findVariables(NameSpace nameSpace)
   {
      List<YoVariable> result = new ArrayList<>();

      for (YoVariable variable : variableList)
      {
         if (variable.getNameSpace().equals(nameSpace))
         {
            result.add(variable);
         }
      }

      return result;
   }

   @Override
   public String toString()
   {
      if (variableList.size() > 10)
      {
         return name + ", contains: " + variableList.size() + " variables.";
      }
      else
      {
         StringBuffer result = new StringBuffer();

         result.append(name);
         result.append(", variables:");

         for (int i = 0; i < variableList.size(); i++)
         {
            result.append("\n");
            result.append(variableList.get(i).toString());
         }

         return result.toString();
      }
   }
}
