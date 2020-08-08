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

import java.io.Serializable;
import java.util.Collections;
import java.util.List;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.yoVariables.exceptions.IllegalNameException;
import us.ihmc.yoVariables.tools.YoTools;

/**
 * Representation of a namespace that is composed of sub-names where typically each sub-name is the
 * name of a {@link YoRegistry} with the parent to child relationship between one sub-name and the
 * following one.
 */
public class YoNamespace implements Serializable
{
   private static final long serialVersionUID = -2584260031738121095L;

   private final String name;
   private final List<String> subNames;

   /**
    * Creates a namespace from a ordered list of names starting with the name of the root.
    *
    * @param subNames is the list of names starting with the root name.
    */
   public YoNamespace(List<String> subNames)
   {
      if (subNames == null || subNames.isEmpty())
         throw new IllegalArgumentException("Cannot create an empty namespace.");

      name = YoTools.joinNames(subNames);
      this.subNames = subNames;
   }

   /**
    * Creates a namespace from a full name of the namespace.
    * <p>
    * The name is the concatenation of all sub names starting with the root name and separated by the
    * {@link YoTools#NAMESPACE_SEPERATOR} character.
    * </p>
    *
    * @param name of the namespace.
    */
   public YoNamespace(String name)
   {
      if (name == null || name.isEmpty())
         throw new IllegalArgumentException("Cannot create an empty namespace.");

      this.name = name;
      subNames = YoTools.splitName(name);

      for (String subName : subNames)
      {
         if (subName.isEmpty())
            throw new IllegalArgumentException("Cannot create a namespace with empty sub-names: " + subNames);
      }
   }

   /**
    * Performs a series of sanity checks for this namespace, such as: verify that there is no redundant
    * or empty sub-names, that the sub-names do not contain illegal characters.
    * 
    * @throws IllegalNameException if the sanity check fails.
    */
   public void checkSanity()
   {
      YoTools.checkNamespaceSanity(this);
   }

   /**
    * Returns this namespace full name, i.e. the combination of all its sub-names jointed by
    * {@value YoTools#NAMESPACE_SEPERATOR_STRING}.
    *
    * @return this namespace's name.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Returns whether this namespace references to the root element, i.e. this namespace is composed of
    * a single sub-name.
    * 
    * @return {@code true} if this namespace references to the root element, {@code false} otherwise.
    */
   public boolean isRoot()
   {
      return subNames.size() == 1;
   }

   /**
    * Returns the first sub-name of this namespace.
    * 
    * @return the name of the root element.
    */
   public String getRootName()
   {
      return subNames.get(0);
   }

   /**
    * Returns the last sub-name of this namespace.
    *
    * @return the short name.
    */
   public String getShortName()
   {
      return subNames.get(size() - 1);
   }

   /**
    * Returns the list of this namespace's sub-names.
    *
    * @return the sub-names for this namespace.
    */
   public List<String> getSubNames()
   {
      return Collections.unmodifiableList(subNames);
   }

   /**
    * Returns the number of sub-names this namespace has.
    * 
    * @return the size of this namespace.
    */
   public int size()
   {
      return subNames.size();
   }

   /**
    * Returns the sub-name at the given position.
    * 
    * @param index the position of the sub-name to return, {@code index} should be in
    *              <tt>[0, this.size()[</tt>
    * @return the sub-name.
    */
   public String getSubName(int index)
   {
      return subNames.get(index);
   }

   /**
    * Returns a new namespace that references the parent element of this namespace, i.e. the parent
    * namespace is equal to this namespace minus the last sub-name.
    * 
    * @return the parent namespace.
    */
   public YoNamespace getParent()
   {
      return removeEnd(1);
   }

   /**
    * Returns a new namespace containing the sub-names located at and after the given index.
    * 
    * @param fromIndex the index of the first sub-name in the sub-namespace to create.
    * @return the sub-namespace.
    */
   public YoNamespace subNamespace(int fromIndex)
   {
      return subNamespace(fromIndex, size());
   }

   /**
    * Returns a new namespace containing the sub-names located at and after the given index.
    * 
    * @param fromIndex the index of the first sub-name in the sub-namespace to create.
    * @param toIndex   the end index (exclusive).
    * @return the sub-namespace.
    */
   public YoNamespace subNamespace(int fromIndex, int toIndex)
   {
      if (fromIndex == 0 && toIndex == size())
         return this;
      if (fromIndex == toIndex)
         return null;
      return new YoNamespace(subNames.subList(fromIndex, toIndex));
   }

   /**
    * Returns a new namespace that is the sub-namespace of {@code this} minus the given number of
    * sub-names starting this namespace.
    * 
    * @param length the number of sub-names to remove for the start.
    * @return the new sub-namespace.
    */
   public YoNamespace removeStart(int length)
   {
      if (length < 0 || length > size())
         throw new IndexOutOfBoundsException("Invalid length: " + length);
      return subNamespace(length);
   }

   /**
    * Will create a new namespace that is the sub-namespace of {@code this} with the provided
    * {@code namespaceToRemove} removed from the start.
    * <p>
    * If {@code this} does not start with the provided namespace or the provided namespace equals
    * {@code this} the method will return {@code null}.
    * </p>
    * <p>
    * If {@code namespaceToRemove} is {@code null} the method will return {@code this}.
    * </p>
    *
    * @param namespaceToRemove is the namespace to remove from the start of {@code this}.
    * @return whether this namespace contains with {@code nameToMatch}.
    */
   public YoNamespace removeStart(YoNamespace namespaceToRemove)
   {
      if (namespaceToRemove == null)
         return this;
      if (equals(namespaceToRemove) || !startsWith(namespaceToRemove))
         return null;
      return removeStart(namespaceToRemove.size());
   }

   /**
    * Returns a new namespace that is the sub-namespace of {@code this} minus the given number of
    * sub-names ending this namespace.
    * 
    * @param length the number of sub-names to remove for the end.
    * @return the new sub-namespace.
    */
   public YoNamespace removeEnd(int length)
   {
      if (length < 0 || length > size())
         throw new IndexOutOfBoundsException("Invalid length: " + length);
      return subNamespace(0, size() - length);
   }

   /**
    * Will create a new namespace that is the sub-namespace of {@code this} with the provided
    * {@code namespaceToRemove} removed from the end.
    * <p>
    * If {@code this} does not end with the provided namespace or the provided namespace equals
    * {@code this} the method will return {@code null}.
    * </p>
    * <p>
    * If {@code namespaceToRemove} is {@code null} the method will return {@code this}.
    * </p>
    *
    * @param namespaceToRemove is the namespace to remove from the end of {@code this}.
    * @return whether this namespace contains with {@code nameToMatch}.
    */
   public YoNamespace removeEnd(YoNamespace namespaceToRemove)
   {
      if (namespaceToRemove == null)
         return this;

      if (equals(namespaceToRemove) || !endsWith(namespaceToRemove))
         return null;

      return removeEnd(namespaceToRemove.size());
   }

   /**
    * Returns a new namespace that is the concatenation of the {@code other} namespace followed with
    * {@code this} namespace.
    * 
    * @param other the other namespace to prepend to this.
    * @return the new namespace [{@code other}, {@code this}].
    */
   public YoNamespace prepend(YoNamespace other)
   {
      return YoTools.concatenate(other, this);
   }

   /**
    * Returns a new namespace that is the concatenation of {@code name} followed with {@code this}
    * namespace.
    * <p>
    * The given {@code name} can either a simple name or the {@code String} representation of a
    * namespace.
    * </p>
    * 
    * @param name the name to prepend to this.
    * @return the new namespace [{@code name}, {@code this}].
    */
   public YoNamespace prepend(String name)
   {
      return YoTools.concatenate(name, this);
   }

   /**
    * Returns a new namespace that is the concatenation of {@code this} namespace followed with the
    * {@code other} namespace.
    * 
    * @param other the other namespace to append to this.
    * @return the new namespace [{@code this}, {@code other}].
    */
   public YoNamespace append(YoNamespace other)
   {
      return YoTools.concatenate(this, other);
   }

   /**
    * Returns a new namespace that is the concatenation of {@code this} namespace followed with
    * {@code name}.
    * <p>
    * The given {@code name} can either a simple name or the {@code String} representation of a
    * namespace.
    * </p>
    * 
    * @param name the name to append to this.
    * @return the new namespace [{@code this}, {@code name}].
    */
   public YoNamespace append(String name)
   {
      return YoTools.concatenate(this, name);
   }

   /**
    * Tests if this namespace ends with the provided namespace.
    * 
    * @param query the namespace to test for.
    * @return {@code true} if this namespace ends with {@code query}, {@code false} otherwise.
    */
   public boolean endsWith(YoNamespace query)
   {
      return endsWith(query, false);
   }

   /**
    * Tests if this namespace ends with the provided namespace.
    * 
    * @param query      the namespace to test for.
    * @param ignoreCase whether to ignore case when performing {@code String} comparison or not.
    * @return {@code true} if this namespace ends with {@code query}, {@code false} otherwise.
    */
   public boolean endsWith(YoNamespace query, boolean ignoreCase)
   {
      if (query.size() > size())
         return false;

      for (int i = 1; i <= query.size(); i++)
      {
         String querySubName = query.subNames.get(query.size() - i);
         String thisSubName = subNames.get(size() - i);

         if (ignoreCase)
         {
            if (!querySubName.equalsIgnoreCase(thisSubName))
               return false;
         }
         else
         {
            if (!querySubName.equals(thisSubName))
               return false;
         }
      }

      return true;
   }

   /**
    * Tests if this namespace ends with the provided name.
    * <p>
    * For example, a namespace with name {@code robot.controller.module} ends with {@code module},
    * {@code controller.module}, and {@code robot.controller.module} but no other string. This means
    * this method will return false for {@code troller.module} or an empty string.
    * </p>
    *
    * @param nameToMatch is the name to check for.
    * @return whether this namespace ends with {@code nameToMatch}.
    */
   public boolean endsWith(String nameToMatch)
   {
      return endsWith(nameToMatch, false);
   }

   /**
    * Tests if this namespace ends with the provided name.
    * <p>
    * For example, a namespace with name {@code robot.controller.module} ends with {@code module},
    * {@code controller.module}, and {@code robot.controller.module} but no other string. This means
    * this method will return false for {@code troller.module} or an empty string.
    * </p>
    *
    * @param nameToMatch is the name to check for.
    * @param ignoreCase  whether to ignore case when performing {@code String} comparison or not.
    * @return whether this namespace ends with {@code nameToMatch}.
    */
   public boolean endsWith(String nameToMatch, boolean ignoreCase)
   {
      if (nameToMatch.length() > name.length())
         return false;

      if (ignoreCase)
      {
         if (!StringUtils.endsWithIgnoreCase(name, nameToMatch))
            return false;
      }
      else
      {
         if (!name.endsWith(nameToMatch))
            return false;
      }

      if (name.length() == nameToMatch.length())
         return true;

      return name.charAt(name.length() - nameToMatch.length() - 1) == YoTools.NAMESPACE_SEPERATOR;
   }

   /**
    * Tests if this namespace starts with the provided namespace.
    * 
    * @param query the namespace to test for.
    * @return {@code true} if this namespace starts with {@code query}, {@code false} otherwise.
    */
   public boolean startsWith(YoNamespace query)
   {
      return startsWith(query, false);
   }

   /**
    * Tests if this namespace starts with the provided namespace.
    * 
    * @param query      the namespace to test for.
    * @param ignoreCase whether to ignore case when performing {@code String} comparison or not.
    * @return {@code true} if this namespace starts with {@code query}, {@code false} otherwise.
    */
   public boolean startsWith(YoNamespace query, boolean ignoreCase)
   {
      if (query.size() > size())
         return false;

      for (int i = 0; i < query.size(); i++)
      {
         if (ignoreCase)
         {
            if (!query.subNames.get(i).equalsIgnoreCase(subNames.get(i)))
               return false;
         }
         else
         {
            if (!query.subNames.get(i).equals(subNames.get(i)))
               return false;
         }
      }

      return true;
   }

   /**
    * Tests if this namespace starts with the provided name.
    * <p>
    * For example, a namespace with name {@code robot.controller.module} starts with {@code robot},
    * {@code robot.controller}, and {@code robot.controller.module} but no other string. This means
    * this method will return false for {@code robot.contro} or an empty string.
    * </p>
    *
    * @param nameToMatch is the name to check for.
    * @return whether this namespace starts with {@code nameToMatch}.
    */
   public boolean startsWith(String nameToMatch)
   {
      return startsWith(nameToMatch, false);
   }

   /**
    * Tests if this namespace starts with the provided name.
    * <p>
    * For example, a namespace with name {@code robot.controller.module} starts with {@code robot},
    * {@code robot.controller}, and {@code robot.controller.module} but no other string. This means
    * this method will return false for {@code robot.contro} or an empty string.
    * </p>
    *
    * @param nameToMatch is the name to check for.
    * @param ignoreCase  whether to ignore case when performing {@code String} comparison or not.
    * @return whether this namespace starts with {@code nameToMatch}.
    */
   public boolean startsWith(String nameToMatch, boolean ignoreCase)
   {
      if (nameToMatch.length() > name.length())
         return false;

      if (ignoreCase)
      {
         if (!StringUtils.startsWithIgnoreCase(name, nameToMatch))
            return false;
      }
      else
      {
         if (!name.startsWith(nameToMatch))
            return false;
      }

      if (name.length() == nameToMatch.length())
         return true;

      return name.charAt(nameToMatch.length()) == YoTools.NAMESPACE_SEPERATOR;
   }

   /**
    * Tests if this namespace contains the provided namespace.
    * 
    * @param query the namespace to test for.
    * @return {@code true} if this namespace contains {@code query}, {@code false} otherwise.
    */
   public boolean contains(YoNamespace query)
   {
      return contains(query, false);
   }

   /**
    * Tests if this namespace contains the provided namespace.
    * 
    * @param query      the namespace to test for.
    * @param ignoreCase whether to ignore case when performing {@code String} comparison or not.
    * @return {@code true} if this namespace contains {@code query}, {@code false} otherwise.
    */
   public boolean contains(YoNamespace query, boolean ignoreCase)
   {
      if (query.size() > size())
         return false;

      int startIndex = -1;

      String queryFirstSubName = query.subNames.get(0);

      for (int i = 0; i < subNames.size() - query.size(); i++)
      {
         boolean areEqual;
         if (ignoreCase)
            areEqual = subNames.get(i).equalsIgnoreCase(queryFirstSubName);
         else
            areEqual = subNames.get(i).equals(queryFirstSubName);

         if (areEqual)
         {
            startIndex = i;
            break;
         }
      }

      if (startIndex == -1)
         return false;

      for (int i = 1; i < query.size(); i++)
      {
         if (ignoreCase)
         {
            if (!query.subNames.get(i).equalsIgnoreCase(subNames.get(startIndex + i)))
               return false;
         }
         else
         {
            if (!query.subNames.get(i).equals(subNames.get(startIndex + i)))
               return false;
         }
      }

      return true;
   }

   /**
    * Tests if this namespace contains the provided name.
    * <p>
    * For example, a namespace with name {@code robot.controller} contains {@code robot},
    * {@code robot.controller}, and {@code controller} but no other string. This means this method will
    * return false for {@code rob} or an empty string.
    * </p>
    *
    * @param nameToMatch is the name to check for.
    * @return whether this namespace contains with {@code nameToMatch}.
    */
   public boolean contains(String nameToMatch)
   {
      return contains(nameToMatch, false);
   }

   /**
    * Tests if this namespace contains the provided name.
    * <p>
    * For example, a namespace with name {@code robot.controller} contains {@code robot},
    * {@code robot.controller}, and {@code controller} but no other string. This means this method will
    * return false for {@code rob} or an empty string.
    * </p>
    *
    * @param nameToMatch is the name to check for.
    * @param ignoreCase  whether to ignore case when performing {@code String} comparison or not.
    * @return whether this namespace contains with {@code nameToMatch}.
    */
   public boolean contains(String nameToMatch, boolean ignoreCase)
   {
      int startIndex;
      if (ignoreCase)
         startIndex = StringUtils.indexOfIgnoreCase(name, nameToMatch);
      else
         startIndex = name.indexOf(nameToMatch);

      if (startIndex == -1)
         return false;

      int endIndex = startIndex + nameToMatch.length();

      if (startIndex > 0 && name.charAt(startIndex - 1) != YoTools.NAMESPACE_SEPERATOR)
         return false;

      if (endIndex < name.length() && name.charAt(endIndex) != YoTools.NAMESPACE_SEPERATOR)
         return false;

      return true;
   }

   @Override
   public int hashCode()
   {
      return name.hashCode();
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof YoNamespace)
         return ((YoNamespace) object).name.equals(name);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return name;
   }
}
