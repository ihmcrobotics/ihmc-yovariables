/*
 * Copyright 2020 Florida Institute for Human and Machine Cognition (IHMC)
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
package us.ihmc.yoVariables.registry;

import java.util.AbstractList;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.yoVariables.exceptions.NameCollisionException;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * {@code YoVariableList} provides a data structure storing a collection of variables while
 * providing an interface to facilitate queries using variable name or namespace.
 */
public class YoVariableList extends AbstractList<YoVariable> implements YoVariableHolder
{
   /** This list's name. */
   private final String name;
   /** List of all the variables added so far to provide index based retrieval. */
   private final List<YoVariable> variableList = new ArrayList<>();
   /** Map from simple name to variables to facilitate name based queries. */
   private final Map<String, List<YoVariable>> simpleNameToVariablesMap = new LinkedHashMap<>();

   /**
    * Creates a new empty list.
    *
    * @param name this list's name for book keeping.
    */
   public YoVariableList(String name)
   {
      this.name = name;
   }

   /**
    * Creates a new list and registers variables.
    *
    * @param name      this list's name for book keeping.
    * @param variables variables to registered during construction.
    */
   public YoVariableList(String name, List<? extends YoVariable> variables)
   {
      this.name = name;
      addAll(variables);
   }

   /**
    * Returns this list's name.
    *
    * @return the name of the list.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Tests whether this list is empty.
    *
    * @return {@code true} if this list is empty.
    */
   @Override
   public boolean isEmpty()
   {
      return variableList.isEmpty();
   }

   /**
    * Returns the number of variables contained in this list.
    */
   @Override
   public int size()
   {
      return variableList.size();
   }

   /**
    * Clears this list.
    */
   @Override
   public void clear()
   {
      variableList.clear();
      simpleNameToVariablesMap.clear();
   }

   /**
    * Returns the variable at the specified position in this list.
    */
   @Override
   public YoVariable get(int index)
   {
      return variableList.get(index);
   }

   /**
    * Returns {@code this}.
    */
   @Override
   public List<YoVariable> getVariables()
   {
      return this;
   }

   /**
    * Adds the given variable to this list.
    * <p>
    * If the variable was already added to this list, nothing happens and this method returns
    * {@code false}.
    * </p>
    *
    * @param variable the new variable to add.
    * @return {@code true} if this operation modified this list.
    * @throws NameCollisionException if a distinct instance of a variable with the same full-name was
    *                                previously added to this list.
    */
   @Override
   public boolean add(YoVariable variable)
   {
      if (!registerVariableInMap(variable))
         return false;
      variableList.add(variable);
      return true;
   }

   /**
    * Adds the given variable to this list at the specified position and shifts the subsequent
    * variables in this list.
    * <p>
    * If the variable was already added to this list, nothing happens.
    * </p>
    * 
    * @param index    index at which the variable is to be inserted.
    * @param variable variable to be inserted.
    * @throws IndexOutOfBoundsException {@inheritDoc}
    */
   @Override
   public void add(int index, YoVariable variable)
   {
      if (index > size() || index < 0)
         throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + size());

      if (!registerVariableInMap(variable))
         return;
      variableList.add(index, variable);
   }

   /**
    * Replaces the variable at the given position in this list with the given variable.
    * <p>
    * If the given variable was previously added to this list, nothing happens and this method returns
    * {@code null}.
    * </p>
    * 
    * @param index    index of the variable to replace.
    * @param variable variable to be stored at the specified position.
    * @return the variable previously registered at the specified position.
    * @throws IndexOutOfBoundsException {@inheritDoc}
    */
   @Override
   public YoVariable set(int index, YoVariable variable)
   {
      YoVariable previousVariable = variableList.get(index);

      if (variable == previousVariable)
         return previousVariable;

      if (!registerVariableInMap(variable))
         return null;

      List<YoVariable> homonyms = simpleNameToVariablesMap.get(previousVariable.getName().toLowerCase());

      if (homonyms != null)
         homonyms.remove(previousVariable);

      variableList.set(index, variable);

      return previousVariable;
   }

   /**
    * Removes the variable at the specified position in this list and shifts any subsequent variables.
    * 
    * @param index the index of the variable to be removed.
    * @return the variable previously stored at the specified position.
    */
   @Override
   public YoVariable remove(int index)
   {
      YoVariable variable = variableList.get(index);
      List<YoVariable> homonyms = simpleNameToVariablesMap.get(variable.getName().toLowerCase());

      if (homonyms != null)
         homonyms.remove(variable);
      variableList.remove(variable);
      return variable;
   }

   /**
    * Attempts to remove the given variable from this list.
    * <p>
    * If this list does not contain the variable, nothing happens.
    * </p>
    * 
    * @param variable the variable to be removed.
    * @return {@code true} if the variable was successfully removed.
    */
   @Override
   public boolean remove(Object variable)
   {
      if (!(variable instanceof YoVariable))
         return false;

      List<YoVariable> homonyms = simpleNameToVariablesMap.get(((YoVariable) variable).getName().toLowerCase());

      if (homonyms == null)
         return false;

      boolean modified = homonyms.remove(variable);
      modified |= variableList.remove(variable);
      return modified;
   }

   private boolean registerVariableInMap(YoVariable variable)
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
            return false;

         // Make sure the variable is unique:
         for (int i = 0; i < homonyms.size(); i++)
         {
            if (homonyms.get(i).getNamespace().equals(variable.getNamespace()))
            {
               throw new NameCollisionException("Name collision with " + variable.getFullNameString());
            }
         }
      }

      homonyms.add(variable);
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public YoVariable findVariable(String namespaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      List<YoVariable> variableList = simpleNameToVariablesMap.get(name.toLowerCase());

      if (variableList == null || variableList.isEmpty())
         return null;

      if (namespaceEnding == null)
      {
         return variableList.get(0);
      }
      else
      {
         for (int i = 0; i < variableList.size(); i++)
         {
            YoVariable candidate = variableList.get(i);

            if (candidate.getNamespace().endsWith(namespaceEnding))
               return candidate;
         }
      }

      return null;
   }

   /** {@inheritDoc} */
   @Override
   public boolean hasUniqueVariable(String namespaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      return countNumberOfVariables(namespaceEnding, name) == 1;
   }

   private int countNumberOfVariables(String parentNamespace, String name)
   {
      List<YoVariable> variableList = simpleNameToVariablesMap.get(name.toLowerCase());

      if (variableList == null || variableList.isEmpty())
         return 0;

      if (parentNamespace == null)
         return variableList.size();

      int count = 0;

      for (int i = 0; i < variableList.size(); i++)
      {
         if (variableList.get(i).getNamespace().endsWith(parentNamespace))
            count++;
      }
      return count;
   }

   /** {@inheritDoc} */
   @Override
   public List<YoVariable> findVariables(String namespaceEnding, String name)
   {
      YoTools.checkNameDoesNotContainSeparator(name);
      List<YoVariable> variableList = simpleNameToVariablesMap.get(name.toLowerCase());

      if (variableList == null || variableList.isEmpty())
         return Collections.emptyList();

      List<YoVariable> result = new ArrayList<>();

      if (namespaceEnding == null)
      {
         result.addAll(variableList);
      }
      else
      {
         for (int i = 0; i < variableList.size(); i++)
         {
            YoVariable candidate = variableList.get(i);

            if (candidate.getNamespace().endsWith(namespaceEnding))
               result.add(candidate);
         }
      }

      return result;
   }

   /** {@inheritDoc} */
   @Override
   public List<YoVariable> findVariables(YoNamespace namespace)
   {
      List<YoVariable> result = new ArrayList<>();

      for (YoVariable variable : variableList)
      {
         if (variable.getNamespace().equals(namespace))
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
