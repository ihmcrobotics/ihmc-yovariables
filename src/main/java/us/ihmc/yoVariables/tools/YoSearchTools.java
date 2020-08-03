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
package us.ihmc.yoVariables.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;
import java.util.regex.Pattern;
import java.util.stream.Stream;

import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This class provides tools for searching variables or parameters in a registry tree.
 */
public class YoSearchTools
{
   /**
    * Recurses the registry subtree to find the first parameter that matches the search criteria.
    * 
    * @param nameSpaceEnding (optional) the namespace of the registry in which the parameter was
    *                        registered. The namespace does not need to be complete, i.e. it does not
    *                        need to contain the name of the registries closest to the root registry.
    *                        If {@code null}, the search is for the parameter name only.
    * @param name            the name of the parameter to retrieve.
    * @param predicate       (optional) additional filter on the parameter to find.
    * @param registry        the registry to search the subtree of.
    * @return the first parameter matching the search criteria, or {@code null} if no such parameter
    *         could be found.
    */
   public static YoParameter findFirstParameter(String nameSpaceEnding, String name, Predicate<YoParameter> predicate, YoRegistry registry)
   {
      Predicate<YoVariable> yoVariablePredicate = YoVariable::isParameter;
      if (predicate != null)
         yoVariablePredicate = yoVariablePredicate.and(variable -> predicate.test(variable.getParameter()));

      YoVariable result = findFirstVariable(nameSpaceEnding, name, yoVariablePredicate, registry);
      if (result == null)
         return null;
      else
         return result.getParameter();
   }

   /**
    * Recurses the registry subtree to find the first variable that matches the search criteria.
    * 
    * @param nameSpaceEnding (optional) the namespace of the registry in which the variable was
    *                        registered. The namespace does not need to be complete, i.e. it does not
    *                        need to contain the name of the registries closest to the root registry.
    *                        If {@code null}, the search is for the variable name only.
    * @param name            the name of the variable to retrieve.
    * @param predicate       (optional) additional filter on the variable to find.
    * @param registry        the registry to search the subtree of.
    * @return the first variable matching the search criteria, or {@code null} if no such variable
    *         could be found.
    */
   public static YoVariable findFirstVariable(String nameSpaceEnding, String name, Predicate<YoVariable> predicate, YoRegistry registry)
   {
      if (nameSpaceEnding == null || registry.getNameSpace().endsWith(nameSpaceEnding))
      {
         YoVariable variable = registry.getVariable(name);
         if (variable != null)
         {
            if (predicate == null || predicate.test(variable))
               return variable;
         }
      }

      for (YoRegistry child : registry.getChildren())
      {
         YoVariable variable = findFirstVariable(nameSpaceEnding, name, predicate, child);
         if (variable != null)
            return variable;
      }

      return null;
   }

   /**
    * Recurses the registry subtree to find all the variables that matches the search criteria.
    * 
    * @param nameSpaceEnding        (optional) the namespace of the registry in which the variable(s)
    *                               was registered. The namespace does not need to be complete, i.e. it
    *                               does not need to contain the name of the registries closest to the
    *                               root registry. If {@code null}, the search is for the variable name
    *                               only.
    * @param name                   the name of the variable(s) to retrieve.
    * @param predicate              (optional) additional filter on the variable(s) to find.
    * @param registry               the registry to search the subtree of.
    * @param matchedVariablesToPack (optional) if provided the found variables are added to this list.
    * @return all the variables matching the search criteria, the list is empty is no such variable
    *         could be found.
    */
   public static List<YoVariable> findVariables(String nameSpaceEnding, String name, Predicate<YoVariable> predicate, YoRegistry registry,
                                                List<YoVariable> matchedVariablesToPack)
   {
      if (matchedVariablesToPack == null)
         matchedVariablesToPack = new ArrayList<>();
      if (nameSpaceEnding == null || registry.getNameSpace().endsWith(nameSpaceEnding))
      {
         YoVariable variable = registry.getVariable(name);
         if (variable != null)
         {
            if (predicate == null || predicate.test(variable))
               matchedVariablesToPack.add(variable);
         }
      }

      for (YoRegistry child : registry.getChildren())
      {
         findVariables(nameSpaceEnding, name, predicate, child, matchedVariablesToPack);
      }
      return matchedVariablesToPack;
   }

   /**
    * Recurses the {@code yoVariableHolder} subtree to find the first variable for which the given
    * {@code filter} returns {@code true}.
    * 
    * @param filter           the filter used as only search criterion.
    * @param yoVariableHolder the variable holder to search the subtree of.
    * @return the first variable matching the search criterion, or {@code null} if no such parameter
    *         could be found.
    */
   public static YoVariable findVariable(Predicate<YoVariable> filter, YoVariableHolder yoVariableHolder)
   {
      for (YoVariable variable : yoVariableHolder.getVariables())
      {
         if (filter.test(variable))
            return variable;
      }

      for (YoVariableHolder child : yoVariableHolder.getChildren())
      {
         YoVariable result = findVariable(filter, child);
         if (result != null)
            return result;
      }

      return null;
   }

   /**
    * Recurses the {@code yoVariableHolder} subtree to find all the variables for which the given
    * {@code filter} returns {@code true}.
    * 
    * @param filter           the filter used as only search criterion.
    * @param yoVariableHolder the variable holder to search the subtree of.
    * @return all the variables matching the search criterion, the list is empty is no such variable
    *         could be found.
    */
   public static List<YoVariable> filterVariables(Predicate<YoVariable> filter, YoVariableHolder yoVariableHolder)
   {
      return filterVariables(filter, yoVariableHolder, null);
   }

   /**
    * Recurses the {@code yoVariableHolder} subtree to find all the variables for which the given
    * {@code filter} returns {@code true}.
    * 
    * @param filter                  the filter used as only search criterion.
    * @param yoVariableHolder        the variable holder to search the subtree of.
    * @param filteredVariablesToPack (optional) if provided the found variables are added to this list.
    * @return all the variables matching the search criterion, the list is empty is no such variable
    *         could be found.
    */
   public static List<YoVariable> filterVariables(Predicate<YoVariable> filter, YoVariableHolder yoVariableHolder, List<YoVariable> filteredVariablesToPack)
   {
      if (filteredVariablesToPack == null)
         filteredVariablesToPack = new ArrayList<>();

      for (YoVariable variable : yoVariableHolder.getVariables())
      {
         if (filter.test(variable))
            filteredVariablesToPack.add(variable);
      }

      for (YoVariableHolder child : yoVariableHolder.getChildren())
      {
         filterVariables(filter, child, filteredVariablesToPack);
      }

      return filteredVariablesToPack;
   }

   /**
    * Recurses the registry subtree to find all the registries for which the given {@code filter}
    * returns {@code true}.
    * 
    * @param filter   the filter used as only search criterion.
    * @param registry the registry to search the subtree of.
    * @return all the registries matching the search criterion, the list is empty is no such registry
    *         could be found.
    */
   public static List<YoRegistry> filterRegistries(Predicate<YoRegistry> filter, YoRegistry registry)
   {
      return filterRegistries(filter, registry, null);
   }

   /**
    * Recurses the {@code yoVariableHolder} subtree to find all the variables for which the given
    * {@code filter} returns {@code true}.
    * 
    * @param filter                   the filter used as only search criterion.
    * @param registry                 the variable holder to search the subtree of.
    * @param filteredRegistriesToPack (optional) if provided the found variables are added to this
    *                                 list.
    * @return all the variables matching the search criterion, the list is empty is no such variable
    *         could be found.
    */
   public static List<YoRegistry> filterRegistries(Predicate<YoRegistry> filter, YoRegistry registry, List<YoRegistry> filteredRegistriesToPack)
   {
      if (filteredRegistriesToPack == null)
         filteredRegistriesToPack = new ArrayList<>();

      if (filter.test(registry))
         filteredRegistriesToPack.add(registry);

      for (YoRegistry child : registry.getChildren())
      {
         filterRegistries(filter, child, filteredRegistriesToPack);
      }

      return filteredRegistriesToPack;
   }

   /**
    * Creates a new filter for searching variable using regular expression(s).
    * <p>
    * The returned filter can be used with {@link #filterVariables(Predicate, YoVariableHolder)}.
    * </p>
    * 
    * @param regularExpressions the regular expressions used to filter variables.
    * @return the filter to use with {@link #filterVariables(Predicate, YoVariableHolder)}.
    */
   public static Predicate<YoVariable> regularExpressionFilter(String... regularExpressions)
   {
      Pattern[] patterns = Stream.of(regularExpressions).map(Pattern::compile).toArray(Pattern[]::new);
      return variable ->
      {
         for (Pattern pattern : patterns)
         {
            if (pattern.matcher(variable.getName()).matches())
               return true;
         }
         return false;
      };
   }

}
