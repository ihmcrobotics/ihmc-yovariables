package us.ihmc.yoVariables.registry;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.function.Predicate;
import java.util.regex.Pattern;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.EnumParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.parameters.LongParameter;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoTools
{
   // TODO: make YoVariables use the same seperator character.
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

   public static DoubleParameter findDoubleParameter(SearchQuery query, YoVariableRegistry registry)
   {
      return (DoubleParameter) findYoParameter(query, DoubleParameter.class::isInstance, registry);
   }

   public static BooleanParameter findBooleanParameter(SearchQuery query, YoVariableRegistry registry)
   {
      return (BooleanParameter) findYoParameter(query, BooleanParameter.class::isInstance, registry);
   }

   public static IntegerParameter findIntegerParameter(SearchQuery query, YoVariableRegistry registry)
   {
      return (IntegerParameter) findYoParameter(query, IntegerParameter.class::isInstance, registry);
   }

   public static LongParameter findLongParameter(SearchQuery query, YoVariableRegistry registry)
   {
      return (LongParameter) findYoParameter(query, LongParameter.class::isInstance, registry);
   }

   public static EnumParameter<?> findEnumParameter(SearchQuery query, YoVariableRegistry registry)
   {
      return (EnumParameter<?>) findYoParameter(query, EnumParameter.class::isInstance, registry);
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public static <E extends Enum<E>> EnumParameter<E> findEnumParameter(SearchQuery query, Class<E> enumType, YoVariableRegistry registry)
   {
      Predicate<YoParameter<?>> predicate = parameter -> parameter instanceof EnumParameter && ((EnumParameter) parameter).getEnumType() == enumType;
      return (EnumParameter<E>) findYoParameter(query, predicate, registry);
   }

   public static YoParameter<?> findYoParameter(SearchQuery query, Predicate<YoParameter<?>> predicate, YoVariableRegistry registry)
   {
      Predicate<YoVariable<?>> yoVariablePredicate = YoVariable::isParameter;
      if (predicate != null)
         yoVariablePredicate = yoVariablePredicate.and(variable -> predicate.test(variable.getParameter()));

      YoVariable<?> result = findYoVariable(query, yoVariablePredicate, registry);
      if (result == null)
         return null;
      else
         return result.getParameter();
   }

   public static YoDouble findYoDouble(SearchQuery query, YoVariableRegistry registry)
   {
      return (YoDouble) findYoVariable(query, YoDouble.class::isInstance, registry);
   }

   public static YoBoolean findYoBoolean(SearchQuery query, YoVariableRegistry registry)
   {
      return (YoBoolean) findYoVariable(query, YoBoolean.class::isInstance, registry);
   }

   public static YoInteger findYoInteger(SearchQuery query, YoVariableRegistry registry)
   {
      return (YoInteger) findYoVariable(query, YoInteger.class::isInstance, registry);
   }

   public static YoLong findYoLong(SearchQuery query, YoVariableRegistry registry)
   {
      return (YoLong) findYoVariable(query, YoLong.class::isInstance, registry);
   }

   public static YoEnum<?> findYoEnum(SearchQuery query, YoVariableRegistry registry)
   {
      return (YoEnum<?>) findYoVariable(query, YoEnum.class::isInstance, registry);
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public static <E extends Enum<E>> YoEnum<E> findYoEnum(SearchQuery query, Class<E> enumType, YoVariableRegistry registry)
   {
      Predicate<YoVariable<?>> predicate = variable -> variable instanceof YoEnum && ((YoEnum) variable).getEnumType() == enumType;
      return (YoEnum<E>) findYoVariable(query, predicate, registry);
   }

   public static YoVariable<?> findYoVariable(SearchQuery query, Predicate<YoVariable<?>> predicate, YoVariableRegistry registry)
   {
      if (query.parentNameSpace == null || registry.getNameSpace().endsWith(query.parentNameSpace))
      {
         YoVariable<?> variable = registry.findVariable(query.name);
         if (variable != null)
         {
            if (predicate == null || predicate.test(variable))
               return variable;
         }
      }

      for (YoVariableRegistry child : registry.getChildren())
      {
         YoVariable<?> variable = findYoVariable(query, predicate, child);
         if (variable != null)
            return variable;
      }

      return null;
   }

   public static List<YoVariable<?>> findYoVariables(SearchQuery query, Predicate<YoVariable<?>> predicate, YoVariableRegistry registry,
                                                     List<YoVariable<?>> matchedVariablesToPack)
   {
      if (matchedVariablesToPack == null)
         matchedVariablesToPack = new ArrayList<>();
      if (query.parentNameSpace == null || registry.getNameSpace().endsWith(query.parentNameSpace))
      {
         YoVariable<?> variable = registry.findVariable(query.name);
         if (variable != null)
         {
            if (predicate == null || predicate.test(variable))
               matchedVariablesToPack.add(variable);
         }
      }

      for (YoVariableRegistry child : registry.getChildren())
      {
         findYoVariables(query, predicate, child, matchedVariablesToPack);
      }
      return matchedVariablesToPack;
   }

   public static class SearchQuery
   {
      private final String parentNameSpace;
      private final String name;

      public SearchQuery(String name)
      {
         int separatorIndex = name.lastIndexOf(NAMESPACE_SEPERATOR_STRING);
         if (separatorIndex == -1)
         {
            parentNameSpace = null;
            this.name = name;
         }
         else
         {
            parentNameSpace = name.substring(0, separatorIndex);
            this.name = name.substring(separatorIndex + 1);
         }
      }

      public SearchQuery(String parentNameSpace, String name)
      {
         if (name.contains(NAMESPACE_SEPERATOR_STRING))
            throw new IllegalNameException(name + " cannot contain '" + NAMESPACE_SEPERATOR_STRING + "'.");

         this.parentNameSpace = parentNameSpace;
         this.name = name;
      }
   }

   public static void printAllVariablesIncludingDescendants(YoVariableRegistry registry, PrintStream out)
   {
      for (YoVariable<?> var : registry.getVariables())
      {
         out.print(var.getFullNameWithNameSpace() + "\n");
      }

      for (YoVariableRegistry child : registry.getChildren())
      {
         printAllVariablesIncludingDescendants(child, out);
      }
   }

   public static void printSizeRecursively(int minVariablesToPrint, int minChildrenToPrint, YoVariableRegistry root)
   {
      List<YoVariableRegistry> registriesOfInterest = new ArrayList<>();
      int totalVariables = collectRegistries(minVariablesToPrint, minChildrenToPrint, root, registriesOfInterest);
      Collections.sort(registriesOfInterest, new Comparator<YoVariableRegistry>()
      {
         @Override
         public int compare(YoVariableRegistry o1, YoVariableRegistry o2)
         {
            if (o1.getNumberOfYoVariables() == o2.getNumberOfYoVariables())
               return 0;
            return o1.getNumberOfYoVariables() > o2.getNumberOfYoVariables() ? -1 : 1;
         }
      });

      System.out.println("");
      LogTools.info("Printing children of " + root.getName() + " registry.");
      System.out.println("Total Number of YoVariables: " + totalVariables);
      System.out.println("Listing registries with at least " + minVariablesToPrint + " variables or at least " + minChildrenToPrint + " children.");
      System.out.println("Sorting by number of variables.");

      for (int registryIdx = 0; registryIdx < registriesOfInterest.size(); registryIdx++)
         printInfo(registriesOfInterest.get(registryIdx));

      System.out.println("");
   }

   private static int collectRegistries(int minVariablesToPrint, int minChildrenToPrint, YoVariableRegistry registry,
                                        List<YoVariableRegistry> registriesOfInterest)
   {
      int variables = registry.getNumberOfYoVariables();
      int children = registry.getChildren().size();

      if (variables >= minVariablesToPrint || children >= minChildrenToPrint)
         registriesOfInterest.add(registry);

      int totalNumberOfVariables = variables;
      for (int childIdx = 0; childIdx < children; childIdx++)
      {
         YoVariableRegistry childRegistry = registry.getChildren().get(childIdx);
         totalNumberOfVariables += collectRegistries(minVariablesToPrint, minChildrenToPrint, childRegistry, registriesOfInterest);
      }

      return totalNumberOfVariables;
   }

   private static void printInfo(YoVariableRegistry registry)
   {
      int variables = registry.getNumberOfYoVariables();
      int children = registry.getChildren().size();

      int maxPropertyLength = 17;
      String variableString = trimStringToLength("Variables: " + variables, maxPropertyLength, "...");
      String childrenString = trimStringToLength("Children: " + children, maxPropertyLength, "...");

      int maxNameLength = 70;
      String name = registry.getClass().getSimpleName() + " " + registry.getNameSpace().getName();
      name = trimStringToLength(name, maxNameLength, "...");

      System.out.println(name + "\t" + variableString + "\t" + childrenString);
   }

   private static String trimStringToLength(String original, int length, String placeholder)
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
      return splitName(fullNameSpace).get(splitName(fullNameSpace).size() - 1);
   }

   public static String joinNames(List<String> subNames)
   {
      if (subNames.stream().anyMatch(subName -> subName.contains(NAMESPACE_SEPERATOR_REGEX)))
      {
         throw new IllegalNameException("A sub name can not contain the seperator string '" + NAMESPACE_SEPERATOR + "'.");
      }
      return String.join(NAMESPACE_SEPERATOR_STRING, subNames);
   }

   public static void checkNameSpaceSanity(NameSpace nameSpace)
   {
      if (nameSpace.getSubNames().stream().anyMatch(subName -> subName.isEmpty()))
      {
         throw new IllegalNameException("Can not construct a namespace with an empty subname.\nNamespace: " + nameSpace.getName());
      }

      if (!joinNames(nameSpace.getSubNames()).equals(nameSpace.getName()))
      {
         throw new IllegalNameException("Can not construct a namespace with inconsistent sub names.\nNamespace: " + nameSpace.getName() + "\nSub Names: "
               + joinNames(nameSpace.getSubNames()));
      }

      if (new HashSet<>(nameSpace.getSubNames()).size() != nameSpace.size())
      {
         throw new IllegalNameException("Can not construct a namespace with duplicate sub names.\nNamespace: " + nameSpace.getName());
      }
   }

   public static NameSpace concatenate(NameSpace nameSpaceA, NameSpace nameSpaceB)
   {
      List<String> subNames = new ArrayList<>(nameSpaceA.size() + nameSpaceB.size());
      subNames.addAll(nameSpaceA.getSubNames());
      subNames.addAll(nameSpaceB.getSubNames());
      return new NameSpace(subNames);
   }
}
