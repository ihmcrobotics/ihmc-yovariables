package us.ihmc.yoVariables.tools;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.function.Function;
import java.util.regex.Pattern;

import org.apache.commons.lang3.StringUtils;
import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.yoVariables.exceptions.IllegalNameException;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoTools
{
   public static final char NAMESPACE_SEPERATOR = '.';
   public static final String NAMESPACE_SEPERATOR_STRING = Character.toString(NAMESPACE_SEPERATOR);
   public static final String NAMESPACE_SEPERATOR_REGEX = Pattern.quote(NAMESPACE_SEPERATOR_STRING);
   public static final String ILLEGAL_CHARACTERS_STRING = "[ .*?@#$%/^&()<>,:{}'\"\\\\]";
   public static final Pattern ILLEGAL_CHARACTERS_PATTERN = Pattern.compile(ILLEGAL_CHARACTERS_STRING);

   public static void checkForIllegalCharacters(String name)
   {
      // String.matches() only matches the whole string ( as if you put ^$ around it ). Use .find() of the Matcher class instead!

      if (ILLEGAL_CHARACTERS_PATTERN.matcher(name).find())
      {
         String message = name + " contains at least one illegal character. Illegal characters: " + ILLEGAL_CHARACTERS_STRING;
         throw new IllegalNameException(message);
      }
   }

   public static void printStatistics(int minVariablesToPrint, int minChildrenToPrint, YoRegistry root)
   {
      printStatistics(minVariablesToPrint, minChildrenToPrint, root, YoTools::getRegistryInfo, System.out);
   }

   public static void printStatistics(int minVariablesToPrint, int minChildrenToPrint, YoRegistry root, Function<YoRegistry, String> registryInfoFunction,
                                      PrintStream printStream)
   {
      List<YoRegistry> registriesOfInterest = new ArrayList<>();

      MutableInt totalVariables = new MutableInt();
      YoSearchTools.filterRegistries(candidate ->
      {
         totalVariables.add(candidate.getVariables().size());
         return candidate.getVariables().size() >= minVariablesToPrint || candidate.getChildren().size() >= minChildrenToPrint;
      }, root);

      Collections.sort(registriesOfInterest, (o1, o2) -> Integer.compare(o2.getNumberOfVariables(), o2.getNumberOfVariables()));

      printStream.println("");
      printStream.println(YoTools.class.getSimpleName() + ": Printing children of " + root.getName() + " registry.");
      printStream.println("Total Number of YoVariables: " + totalVariables.intValue());
      printStream.println("Listing registries with at least " + minVariablesToPrint + " variables or at least " + minChildrenToPrint + " children.");
      printStream.println("Sorting by number of variables.");

      registriesOfInterest.forEach(registry -> printStream.println(getRegistryInfo(registry)));

      printStream.println("");
   }

   public static String getRegistryInfo(YoRegistry registry)
   {
      int maxPropertyLength = 17;
      int maxNameLength = 70;
      return getRegistryInfo(registry, maxPropertyLength, maxNameLength);
   }

   public static String getRegistryInfo(YoRegistry registry, int maxPropertyLength, int maxNameLength)
   {
      int variables = registry.getNumberOfVariables();
      int children = registry.getChildren().size();

      String variableString = trimOrPadToLength("Variables: " + variables, maxPropertyLength, "...");
      String childrenString = trimOrPadToLength("Children: " + children, maxPropertyLength, "...");

      String name = registry.getClass().getSimpleName() + " " + registry.getNameSpace().getName();
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

   public static List<String> splitName(String name)
   {
      return Arrays.asList(name.split(NAMESPACE_SEPERATOR_REGEX, -1));
   }

   public static String toShortName(String fullNameSpace)
   {
      int separatorIndex = fullNameSpace.lastIndexOf(NAMESPACE_SEPERATOR_STRING);
      if (separatorIndex == -1)
         return fullNameSpace;
      else
         return fullNameSpace.substring(separatorIndex + 1);
   }

   public static String joinNames(List<String> subNames)
   {
      if (subNames.stream().anyMatch(subName -> subName.contains(NAMESPACE_SEPERATOR_REGEX)))
      {
         throw new IllegalNameException("A sub name can not contain the seperator string '" + NAMESPACE_SEPERATOR + "'.");
      }
      return String.join(NAMESPACE_SEPERATOR_STRING, subNames);
   }

   public static void checkNameDoesNotContainSeparator(String name)
   {
      if (name.contains(NAMESPACE_SEPERATOR_STRING))
         throw new IllegalNameException("The name cannot contain '" + NAMESPACE_SEPERATOR_STRING + "'. Was: " + name);
   }

   public static void checkNameSpaceSanity(NameSpace nameSpace)
   {
      if (nameSpace.getSubNames().stream().anyMatch(subName -> subName.isEmpty()))
      {
         throw new IllegalNameException("The namespace has 1+ empty subname.\nNamespace: " + nameSpace.getName());
      }

      if (!joinNames(nameSpace.getSubNames()).equals(nameSpace.getName()))
      {
         throw new IllegalNameException("The namespace has inconsistent sub names.\nNamespace: " + nameSpace.getName() + "\nSub Names: "
               + joinNames(nameSpace.getSubNames()));
      }

      if (new HashSet<>(nameSpace.getSubNames()).size() != nameSpace.size())
      {
         throw new IllegalNameException("The namespace has duplicate sub names.\nNamespace: " + nameSpace.getName());
      }
   }

   public static NameSpace concatenate(NameSpace nameSpaceA, NameSpace nameSpaceB)
   {
      List<String> subNames = new ArrayList<>(nameSpaceA.size() + nameSpaceB.size());
      subNames.addAll(nameSpaceA.getSubNames());
      subNames.addAll(nameSpaceB.getSubNames());
      return new NameSpace(subNames);
   }

   public static NameSpace concatenate(NameSpace nameSpace, String name)
   {
      List<String> splitName = splitName(name);
      List<String> subNames = new ArrayList<>(nameSpace.size() + splitName.size());
      subNames.addAll(nameSpace.getSubNames());
      subNames.addAll(splitName);
      return new NameSpace(subNames);
   }

   public static NameSpace concatenate(String name, NameSpace nameSpace)
   {
      List<String> splitName = splitName(name);
      List<String> subNames = new ArrayList<>(splitName.size() + nameSpace.size());
      subNames.addAll(splitName);
      subNames.addAll(nameSpace.getSubNames());
      return new NameSpace(subNames);
   }

   public static NameSpace concatenate(String nameA, String nameB)
   {
      List<String> splitNameA = splitName(nameA);
      List<String> splitNameB = splitName(nameB);
      List<String> subNames = new ArrayList<>(splitNameA.size() + splitNameB.size());
      subNames.addAll(splitNameA);
      subNames.addAll(splitNameB);
      return new NameSpace(subNames);
   }
}
