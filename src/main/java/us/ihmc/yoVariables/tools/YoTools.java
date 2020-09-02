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

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.yoVariables.exceptions.IllegalNameException;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This class provides general tools used in classes such as {@code YoNamespace},
 * {@code YoRegistry}, and {@code YoVariable}.
 */
public class YoTools
{
   /**
    * Character used to separate sub-names in a namespace when the namespace is represented as a single
    * {@code String}.
    */
   public static final char NAMESPACE_SEPERATOR = '.';
   /**
    * Character used to separate sub-names in a namespace when the namespace is represented as a single
    * {@code String}.
    */
   public static final String NAMESPACE_SEPERATOR_STRING = Character.toString(NAMESPACE_SEPERATOR);
   /** Namespace separation as a regular expression. */
   public static final String NAMESPACE_SEPERATOR_REGEX = Pattern.quote(NAMESPACE_SEPERATOR_STRING);
   /**
    * The regular expression for searching any illegal character that can not be used in the name of a
    * {@link YoRegistry} or a {@link YoVariable}.
    * <p>
    * Any punctuation character is illegal except for the underscore "_" and hyphen "-" characters that
    * are permitted, and spaces are also illegal. List of illegal characters:
    * <tt> `~.*!?@#$%/^&()<>,:;{}'"\=+|</tt>
    * </p>
    */
   public static final String ILLEGAL_CHARACTERS_REGEX = "[ `~.*!?@#$%/^&()<>,:;{}'\"\\\\=+|]";
   /**
    * The regular expression of illegal characters as a {@link Pattern} which can be used to create a
    * {@link Matcher} can be easily setup to perform a thorough search for illegal characters.
    */
   public static final Pattern ILLEGAL_CHARACTERS_PATTERN = Pattern.compile(ILLEGAL_CHARACTERS_REGEX);

   /**
    * Checks that the given {@code name} does not contain any illegal character.
    * <p>
    * Any punctuation character is illegal except for the underscore "_" and hyphen "-" characters that
    * are permitted, and spaces are also illegal. List of illegal characters:
    * <tt> `~.*!?@#$%/^&()<>,:;{}'"\=+|</tt>
    * </p>
    * 
    * @param name the {@code String} to validate.
    * @throws IllegalNameException if the name contains at least one illegal character.
    */
   public static void checkForIllegalCharacters(String name)
   {
      // String.matches() only matches the whole string ( as if you put ^$ around it ). Use .find() of the Matcher class instead!

      if (ILLEGAL_CHARACTERS_PATTERN.matcher(name).find())
      {
         String message = name + " contains at least one illegal character. Illegal characters: " + ILLEGAL_CHARACTERS_REGEX;
         throw new IllegalNameException(message);
      }
   }

   /**
    * Recurses the sub-tree starting at the given registry and prints to {@code System.out} the number
    * of variables and children for each registry.
    * <p>
    * Before printing the statistics, the registries are first filtered such that small registries with
    * few variables and few children are omitted, and then sorted based on the number of variables for
    * each registry in a descending order.
    * </p>
    * 
    * @param minVariablesToPrint first filter threshold, if a registry has at least this number of
    *                            variables registered its statistics are printed.
    * @param minChildrenToPrint  second filter threshold, if a registry has at least this number of
    *                            children registered its statistics are printed.
    * @param root                the root of the sub-tree to print statistics of.
    */
   public static void printStatistics(int minVariablesToPrint, int minChildrenToPrint, YoRegistry root)
   {
      printStatistics(minVariablesToPrint, minChildrenToPrint, root, YoTools::getRegistryInfo, System.out);
   }

   /**
    * Recurses the sub-tree starting at the given registry and prints to the given stream the number of
    * variables and children for each registry.
    * <p>
    * Before printing the statistics, the registries are first filtered such that small registries with
    * few variables and few children are omitted, and then sorted based on the number of variables for
    * each registry in a descending order.
    * </p>
    * 
    * @param minVariablesToPrint  first filter threshold, if a registry has at least this number of
    *                             variables registered its statistics are printed.
    * @param minChildrenToPrint   second filter threshold, if a registry has at least this number of
    *                             children registered its statistics are printed.
    * @param root                 the root of the sub-tree to print statistics of.
    * @param registryInfoFunction function used when printing statistics for one registry.
    * @param printStream          the stream to print to.
    */
   public static void printStatistics(int minVariablesToPrint, int minChildrenToPrint, YoRegistry root, Function<YoRegistry, String> registryInfoFunction,
                                      PrintStream printStream)
   {
      printStatistics(candidate -> candidate.getVariables().size() >= minVariablesToPrint || candidate.getChildren().size() >= minChildrenToPrint,
                      root,
                      registryInfoFunction,
                      printStream);
   }

   /**
    * Recurses the sub-tree starting at the given registry and prints to the given stream the number of
    * variables and children for each registry.
    * <p>
    * Before printing the statistics, the registries are first filtered using the given filter, and
    * then sorted based on the number of variables for each registry in a descending order.
    * </p>
    * 
    * @param filter               the filter to select a subset of the sub-tree.
    * @param root                 the root of the sub-tree to print statistics of.
    * @param registryInfoFunction function used when printing statistics for one registry.
    * @param printStream          the stream to print to.
    */
   public static void printStatistics(Predicate<YoRegistry> filter, YoRegistry root, Function<YoRegistry, String> registryInfoFunction, PrintStream printStream)
   {
      List<YoRegistry> registriesOfInterest = YoSearchTools.filterRegistries(filter, root);
      Collections.sort(registriesOfInterest, (o1, o2) -> Integer.compare(o2.getNumberOfVariables(), o1.getNumberOfVariables()));
      printStream.println(YoTools.class.getSimpleName() + ": Printing descendants of " + root.getName() + " registry.");
      printStream.println("Total number of variables: " + root.getNumberOfVariablesDeep());
      printStream.println("Sorting by number of variables.");
      registriesOfInterest.forEach(registry -> printStream.println(registryInfoFunction.apply(registry)));
   }

   /**
    * Returns as a {@code String} information about the given registry, that are number of variables
    * and children.
    * <p>
    * The resulting {@code String} is formatted to facilitate reading information printed for multiple
    * registries.
    * </p>
    * 
    * @param registry the registry to get information of.
    * @return the {@code String} containing information about the registry.
    * @see #getRegistryInfo(YoRegistry, int)
    */
   public static String getRegistryInfo(YoRegistry registry)
   {
      int maxNameLength = 70;
      return getRegistryInfo(registry, maxNameLength);
   }

   /**
    * Returns as a {@code String} information about the given registry, that are number of variables
    * and children.
    * <p>
    * The resulting {@code String} is formatted to facilitate reading information printed for multiple
    * registries.
    * </p>
    * 
    * @param registry      the registry to get information of.
    * @param maxNameLength used for alignment when printing info for multiple registries. If the name
    *                      of a registry is longer than this length, the name is trimmed down and an
    *                      ellipsis is added to indicate that it is trimmed.
    * @return the {@code String} containing information about the registry.
    */
   public static String getRegistryInfo(YoRegistry registry, int maxNameLength)
   {
      int variables = registry.getNumberOfVariables();
      int children = registry.getChildren().size();
      int maxPropertyLength = 17; // "Variables: " is 11 chars leaving 6 for the integer.

      String variableString = trimOrPadToLength("Variables: " + variables, maxPropertyLength, "...");
      String childrenString = trimOrPadToLength("Children: " + children, maxPropertyLength, "...");

      String name = registry.getClass().getSimpleName() + " " + registry.getNamespace().getName();
      name = trimOrPadToLength(name, maxNameLength, "...");

      return name + "\t" + variableString + "\t" + childrenString;
   }

   private static String trimOrPadToLength(String original, int length, String placeholder)
   {
      int chararcters = original.length();
      int placeholderLength = placeholder.length();

      if (chararcters > length)
         return original.substring(0, length - placeholderLength) + placeholder;
      else
         return StringUtils.rightPad(original, length, " ");
   }

   /**
    * Assuming the given name may represent a namespace, this method splits at every occurrence of
    * {@link #NAMESPACE_SEPERATOR}. The namespace separator, if present, is removed from the result.
    * 
    * @param name the {@code String} to process and split into sub-names.
    * @return the list of sub-names.
    */
   public static List<String> splitName(String name)
   {
      return Arrays.asList(name.split(NAMESPACE_SEPERATOR_REGEX, -1));
   }

   /**
    * Assuming the given name may represent a namespace, this method isolate the substring starting
    * right after the last occurrence of {@link #NAMESPACE_SEPERATOR}. If the
    * {@link #NAMESPACE_SEPERATOR} could not be found, the name is returned.
    * 
    * @param name the {@code String} to process and extract the short name from.
    * @return the short name, i.e. the last sub-name if name represents a namespace, or {@code name}
    *         otherwise.
    */
   public static String toShortName(String name)
   {
      int separatorIndex = name.lastIndexOf(NAMESPACE_SEPERATOR_STRING);
      if (separatorIndex == -1)
         return name;
      else
         return name.substring(separatorIndex + 1);
   }

   /**
    * Joins the given sub-names into a single {@code String} and insert {@link #NAMESPACE_SEPERATOR} as
    * a separator between each pair of sub-names.
    * 
    * @param subNames the sub-names to join.
    * @return the {@code String} representing the namespace described by the given sub-names.
    */
   public static String joinNames(List<String> subNames)
   {
      return String.join(NAMESPACE_SEPERATOR_STRING, subNames);
   }

   /**
    * Checks that the given name does not contain {@link #NAMESPACE_SEPERATOR}.
    * 
    * @param name the name to validate.
    * @throws IllegalNameException if the given {@code name} contains the namespace separator.
    */
   public static void checkNameDoesNotContainSeparator(String name)
   {
      if (name.contains(NAMESPACE_SEPERATOR_STRING))
         throw new IllegalNameException("The name cannot contain '" + NAMESPACE_SEPERATOR_STRING + "'. Was: " + name);
   }

   /**
    * Performs a suite of checks on the given namespace to assert that it represents a proper
    * namespace.
    * <p>
    * The following checks are performed:
    * <ul>
    * <li>sub-name cannot be empty,
    * <li>sub-name cannot contain the namespace separator {@link #NAMESPACE_SEPERATOR},
    * <li>the sub-names and name of the given {@code namespace} have to be consistent,
    * <li>each sub-name is unique.
    * </ul>
    * </p>
    * 
    * @param namespace the namespace to validate.
    * @throws IllegalNameException if the given {@code namespace} failed any of the checks mentioned
    *                              above.
    */
   public static void checkNamespaceSanity(YoNamespace namespace)
   {
      if (namespace.getSubNames().stream().anyMatch(subName -> subName.isEmpty()))
      {
         throw new IllegalNameException("The namespace has 1+ empty subname.\nNamespace: " + namespace.getName());
      }

      if (namespace.getSubNames().stream().anyMatch(subName -> subName.contains(NAMESPACE_SEPERATOR_REGEX)))
      {
         throw new IllegalNameException("A sub-name can not contain the seperator string '" + NAMESPACE_SEPERATOR + "'.");
      }

      if (!joinNames(namespace.getSubNames()).equals(namespace.getName()))
      {
         throw new IllegalNameException("The namespace has inconsistent sub-names.\nNamespace: " + namespace.getName() + "\nSub-names: "
               + joinNames(namespace.getSubNames()));
      }

      if (new HashSet<>(namespace.getSubNames()).size() != namespace.size())
      {
         throw new IllegalNameException("The namespace has duplicate sub-names.\nNamespace: " + namespace.getName());
      }
   }

   /**
    * Concatenates the two namespaces into a new namespace: [{@code namespaceA}, {@code namespaceB}].
    * 
    * @param namespaceA the first namespace.
    * @param namespaceB the second namespace.
    * @return the new namespace that starts with {@code namespaceA} and ends with {@code namespaceB}.
    */
   public static YoNamespace concatenate(YoNamespace namespaceA, YoNamespace namespaceB)
   {
      List<String> subNames = new ArrayList<>(namespaceA.size() + namespaceB.size());
      subNames.addAll(namespaceA.getSubNames());
      subNames.addAll(namespaceB.getSubNames());
      return new YoNamespace(subNames);
   }

   /**
    * Appends the given {@code name} to the given {@code namespace} and returns the results as a new
    * namespace.
    * 
    * @param namespace the namespace.
    * @param name      the name to append. It can represent another namespace, i.e. it can contain
    *                  {@link #NAMESPACE_SEPERATOR}.
    * @return the new namespace that starts with {@code namespace} and ends with {@code name}.
    */
   public static YoNamespace concatenate(YoNamespace namespace, String name)
   {
      List<String> splitName = splitName(name);
      List<String> subNames = new ArrayList<>(namespace.size() + splitName.size());
      subNames.addAll(namespace.getSubNames());
      subNames.addAll(splitName);
      return new YoNamespace(subNames);
   }

   /**
    * Appends the given {@code namespace} to the given {@code name} and returns the results as a new
    * namespace.
    * 
    * @param name      the name. It can represent another namespace, i.e. it can contain
    *                  {@link #NAMESPACE_SEPERATOR}.
    * @param namespace the namespace to append.
    * @return the new namespace that starts with {@code name} and ends with {@code namespace}.
    */
   public static YoNamespace concatenate(String name, YoNamespace namespace)
   {
      List<String> splitName = splitName(name);
      List<String> subNames = new ArrayList<>(splitName.size() + namespace.size());
      subNames.addAll(splitName);
      subNames.addAll(namespace.getSubNames());
      return new YoNamespace(subNames);
   }

   /**
    * Creates a new namespace that starts with {@code nameA} and ends with {@code nameB}.
    * 
    * @param nameA the first name. It can represent another namespace, i.e. it can contain
    *              {@link #NAMESPACE_SEPERATOR}.
    * @param nameB the second name. It can represent another namespace, i.e. it can contain
    *              {@link #NAMESPACE_SEPERATOR}.
    * @return the new namespace that starts with {@code nameA} and ends with {@code nameB}.
    */
   public static YoNamespace concatenate(String nameA, String nameB)
   {
      List<String> splitNameA = splitName(nameA);
      List<String> splitNameB = splitName(nameB);
      List<String> subNames = new ArrayList<>(splitNameA.size() + splitNameB.size());
      subNames.addAll(splitNameA);
      subNames.addAll(splitNameB);
      return new YoNamespace(subNames);
   }
}
